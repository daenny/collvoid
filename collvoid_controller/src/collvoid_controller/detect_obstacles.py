#!/usr/bin/env python
__author__ = 'danielclaes'
import rospy
from geometry_msgs.msg import PolygonStamped, PointStamped, Point
from collvoid_msgs.msg import PoseTwistWithCovariance
from collvoid_srvs.srv import GetObstacles
from sensor_msgs.msg import LaserScan, PointCloud
import tf
import tf.transformations
import cv2
import cv2.cv
import numpy as np

GLOBAL_FRAME = rospy.get_param('global_frame', '/map')
BASE_FRAME = rospy.get_param('base_frame', 'base_link')

def convex_hull(points):
    """Computes the convex hull of a set of 2D points.

    Input: an iterable sequence of (x, y) pairs representing the points.
    Output: a list of vertices of the convex hull in counter-clockwise order,
      starting from the vertex with the lexicographically smallest coordinates.
    Implements Andrew's monotone chain algorithm. O(n log n) complexity.
    """

    # Sort the points lexicographically (tuples are compared lexicographically).
    # Remove duplicates to detect the case we have just one unique point.
    points = sorted(set(points))

    # Boring case: no points or a single point, possibly repeated multiple times.
    if len(points) <= 1:
        return points

    # 2D cross product of OA and OB vectors, i.e. z-component of their 3D cross product.
    # Returns a positive value, if OAB makes a counter-clockwise turn,
    # negative for clockwise turn, and zero if the points are collinear.
    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    # Build lower hull
    lower = []
    for p in points:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)

    # Build upper hull
    upper = []
    for p in reversed(points):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)

    # Concatenation of the lower and upper hulls gives the convex hull.
    # Last point of each list is omitted because it is repeated at the beginning of the other list.
    return lower[:-1] + upper[:-1]


def poly_area_2d(pts):
    lines = np.hstack([pts,np.roll(pts,-1,axis=0)])
    area = 0.5*abs(sum(x1*y2-x2*y1 for x1,y1,x2,y2 in lines))
    return area


def laser_to_img(x, y, max_x, max_y, img_size_x, img_size_y):
    img_x = int(img_size_x * x/max_x) + img_size_x/2
    img_y = int(img_size_y * y/max_y) + img_size_y/2
    img_x = max(min(img_size_x-1, img_x), 0)
    img_y = max(min(img_size_y-1, img_y), 0)
    return img_x, img_y


IMG_SIZE_X = 800
IMG_SIZE_Y = 800


class DetectObstacles(object):
    def __init__(self):
        self.tf_listener = tf.TransformListener()
        rospy.sleep(0.5)
        self.__angle_min = 0.0
        self.__angle_max = 0.0
        self.__cos_sin_map = np.array([[]])

        self.current_obstacles = []
        self.polygon_pub = rospy.Publisher('obstacle_polygons', PolygonStamped, queue_size=1)
        rospy.Subscriber('base_scan', LaserScan, self.cb_laser)
        rospy.Subscriber('/position_share', PoseTwistWithCovariance, self.cb_pose_twist)
        rospy.Service('get_current_obstacles', GetObstacles, self.cb_get_obstacles_srv)

    def cb_get_obstacles_srv(self, req):
        return {'obstacles': self.current_obstacles}

    def cb_laser(self, msg):
        """

        :param msg:
        :type msg:LaserScan
        :return:
        :rtype:
        """
        N = len(msg.ranges)
        ranges = np.array(msg.ranges)
        ranges = np.array([ranges, ranges])
        max_range = msg.range_max

        if (self.__cos_sin_map.shape[1] != N or
            self.__angle_min != msg.angle_min or
            self.__angle_max != msg.angle_max):
            rospy.logdebug("No precomputed map given. Computing one.")

            self.__angle_min = msg.angle_min
            self.__angle_max = msg.angle_max

            cos_map = [np.cos(msg.angle_min + i * msg.angle_increment)
                    for i in range(N)]
            sin_map = [np.sin(msg.angle_min + i * msg.angle_increment)
                    for i in range(N)]

            self.__cos_sin_map = np.array([cos_map, sin_map])

        output = ranges * self.__cos_sin_map
        ranges = np.array(msg.ranges)

        # output = output[[ranges < max_range],[ranges<max_range]]

        output_points = [[output[0][x], output[1][x]] for x in range(0, len(output[0]))]

        output_img = (output/max_range * 400 + 400).astype(int)
        output_img_points = np.array([[output_img[0][x], output_img[1][x]] for x in range(0, len(output_img[0]))])
        output_img_points = output_img_points[ranges < max_range]

        current_obstacles = []
        current_list = []

        for p in output_img_points:
            current_list = self.add_point(p, current_list, current_obstacles)
        h = cv2.convexHull(np.array(current_list))
        current_obstacles.append(h)

        current_obstacles = self.filter_obstacles(current_obstacles)

        self.remap_points_to_polygons(current_obstacles, max_range)

        img = np.zeros([IMG_SIZE_Y, IMG_SIZE_X], dtype=np.uint8)

        for p in output_img_points:
            img[p[1]][p[0]] = 255

        return_img = img.copy()
        for obst in current_obstacles:
            # output_obst = (obst/max_range * 400 + 200).astype(int)
            cv2.drawContours(return_img, [obst], 0, 255, 2)

        cv2.imshow("laser", return_img)
        cv2.waitKey(5)
        # cv2.convexHull(points)

    def filter_obstacles(self, current_obstacles):
        remove_list = []
        for idx, obst in enumerate(current_obstacles):
            if len(obst) < 2 or cv2.contourArea(obst) < 20:
                remove_list.append(idx)
        remove_list.reverse()
        for rm in remove_list:
            current_obstacles.pop(rm)
        return current_obstacles

    def add_point(self, point, current_list, current_obstacles):
        if len(current_list) > 0:
            if np.linalg.norm(np.array(point) - np.array(current_list[-1])) > 30:
                h = cv2.convexHull(np.array(current_list))
                current_obstacles.append(h)
                current_list = [point]
                return current_list

        current_list.append(point)
        h = cv2.convexHull(np.array(current_list))
        rect = cv2.minAreaRect(h)
        box = cv2.cv.BoxPoints(rect)
        box = np.int32(box)
        vec1 = box[1] - box[2]
        vec2 = box[2] - box[3]

        axis = (np.linalg.norm(vec1), np.linalg.norm(vec2))
        if min(axis) > 20:
            current_list.pop()
            h = cv2.convexHull(np.array(current_list))
            current_obstacles.append(h)
            current_list = [point]
        return current_list

    def remap_points_to_polygons(self, obstacles, max_range):
        now = rospy.Time.now()
        self.current_obstacles = []
        for obst in obstacles:
            poly = PolygonStamped()
            poly.header.stamp = now
            poly.header.frame_id = BASE_FRAME
            pc = PointCloud()
            pc.header.stamp = now
            pc.header.frame_id = BASE_FRAME
            for p in obst:
                t = Point()
                t.x = (p[0][0]-400.)/400. * max_range
                t.y = (p[0][1]-400.)/400. * max_range
                pc.points.append(t)

            # self.tf_listener.transformPointCloud(GLOBAL_FRAME, pc)
            poly.polygon.points = pc.points
            self.current_obstacles.append(poly)
            self.polygon_pub.publish(poly)

    def cb_pose_twist(self, msg):
        pass


if __name__ == '__main__':
    rospy.init_node('detect_obstacles')
    detector = DetectObstacles()
    rospy.spin()

