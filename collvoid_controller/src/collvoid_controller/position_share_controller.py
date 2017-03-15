#!/usr/bin/env python
import random

from sensor_msgs.msg import PointCloud2, LaserScan
import sensor_msgs.point_cloud2 as pcl2
from std_msgs.msg import Header
from std_srvs.srv import Empty
import rospy
from geometry_msgs.msg import Pose, Quaternion
from collvoid_msgs.msg import PoseTwistWithCovariance
from collvoid_srvs.srv import GetNeighbors, GetNeighborsResponse
from socket import gethostname
import tf
import tf.transformations
import numpy as np
from threading import Lock

__author__ = 'danielclaes'


def quat_array_from_msg(o):
    return np.array([o.x, o.y, o.z, o.w])


def make_rotation_transformation(angle, origin=(0, 0)):
    """
    used to create a fake sensor to add neighbours to the costmap
    :param angle:
    :param origin:
    :return:
    """
    cos_theta, sin_theta = np.math.cos(angle), np.math.sin(angle)
    x0, y0 = origin

    def xform(point):
        x, y = point[0] - x0, point[1] - y0
        return (x * cos_theta - y * sin_theta + x0,
                x * sin_theta + y * cos_theta + y0, Z_HEIGHT)

    return xform


class PositionShareController(object):
    def __init__(self):
        self.position_share_pub = rospy.Publisher('/position_share', PoseTwistWithCovariance, queue_size=1)
        # Set the name
        self.name = rospy.get_namespace()
        if self.name == "/":
            self.name = gethostname()
        else:
            self.name = self.name.replace('/', '')

        self.name = rospy.get_param('~name', self.name)
        rospy.loginfo("Position Share started with name: %s", self.name)

        self.neighbors = {}
        self.neighbors_lock = Lock()
        self.me = None

        self.sensor_link = rospy.get_param("~base_frame_id", rospy.get_namespace()[1:] + "base_link")

        self.cloud_header = Header()
        self.cloud_header.frame_id = self.sensor_link
        self.point_cloud_pub = rospy.Publisher('stationary_robots', PointCloud2, queue_size=2)

        self.clearing_laser_pub = rospy.Publisher('clearing_scan', LaserScan, queue_size=1)
        self.clearing_laser_scan = LaserScan()
        self.clearing_laser_scan.header.frame_id = self.sensor_link
        self.clearing_laser_scan.angle_min = -np.math.pi
        self.clearing_laser_scan.angle_max = np.math.pi
        num_scans = 600
        self.clearing_laser_scan.angle_increment = np.math.pi * 2 / num_scans
        self.clearing_laser_scan.range_min = 0
        self.clearing_laser_scan.range_max = 5
        self.clearing_laser_scan.ranges = [3.0] * num_scans

        # self.reset_srv = rospy.ServiceProxy('move_base/DWAPlannerROS/clear_local_costmap', Empty, persistent=True)
        # rospy.wait_for_service('move_base/DWAPlannerROS/clear_local_costmap', timeout=10.)
        rospy.Subscriber('/position_share', PoseTwistWithCovariance, self.position_share_cb, queue_size=1)
        rospy.Service('get_neighbors', GetNeighbors, self.get_neighbors_cb)

    def position_share_cb(self, msg):
        """
        :param msg:
        :type msg: PoseTwistWithCovariance
        """
        with self.neighbors_lock:
            if msg.robot_id == self.name:
                self.me = msg
                return
            if msg.robot_id not in self.neighbors:
                self.neighbors[msg.robot_id] = {}
                self.neighbors[msg.robot_id]['stationary'] = False
            robot = self.neighbors[msg.robot_id]
            robot['robot_id'] = msg.robot_id
            robot['controlled'] = msg.controlled
            robot['twist'] = msg.twist
            robot['position'] = msg.pose
            robot['footprint'] = msg.footprint
            robot['radius'] = msg.radius
            robot['holo_robot'] = msg.holo_robot
            robot['holo_speed'] = msg.holonomic_velocity
            robot['radius'] = msg.radius
            robot['last_seen'] = msg.header.stamp  # or rospy.Time.now()

    def get_neighbors_cb(self, req):
        with self.neighbors_lock:
            response = GetNeighborsResponse()
            time = rospy.Time.now()
            for name in self.neighbors:
                if (time - self.neighbors[name]['last_seen']).to_sec() < last_seen_threshold:
                    response.neighbors.append(self.create_msg(self.neighbors[name], time))
            return response

    def create_msg(self, robot, time):
        msg = PoseTwistWithCovariance()
        msg.header.stamp = time
        msg.header.frame_id = global_frame
        msg.robot_id = robot['robot_id']
        msg.controlled = robot['controlled']
        msg.holo_robot = robot['holo_robot']
        msg.twist = robot['twist']
        msg.pose.pose = self.predict_pose(robot, time)
        msg.footprint = robot['footprint']
        msg.radius = robot['radius']
        msg.holo_robot = robot['holo_robot']
        msg.holonomic_velocity = robot['holo_speed']
        msg.radius = robot['radius']
        return msg

    def publish_static_robots(self, force_clear):
        with self.neighbors_lock:
            time = rospy.Time.now()

            cloud_points = []
            if self.me is None or (time - self.me.header.stamp).to_sec() > last_seen_threshold:
                return

            my_pose = self.me.pose.pose
            _, _, my_theta = tf.transformations.euler_from_quaternion(quat_array_from_msg(my_pose.orientation))

            change = False
            for name in self.neighbors:
                if (time - self.neighbors[name]['last_seen']).to_sec() < last_seen_threshold \
                        and ((abs(self.neighbors[name]['twist'].twist.linear.x) < 0.05 and abs(
                            self.neighbors[name]['twist'].twist.linear.y) < 0.05)):

                    if not self.neighbors[name]['stationary']:
                        change = True
                        self.neighbors[name]['stationary'] = True

                    cur_pose = self.neighbors[name]['position'].pose
                    _, _, cur_theta = tf.transformations.euler_from_quaternion(quat_array_from_msg(cur_pose.orientation))

                    relative_pose_x = cur_pose.position.x - my_pose.position.x
                    relative_pose_y = cur_pose.position.y - my_pose.position.y

                    xform_foot = make_rotation_transformation(cur_theta - my_theta)
                    xform_to_baselink = make_rotation_transformation(-my_theta)
                    pos_rel = xform_to_baselink((relative_pose_x, relative_pose_y))

                    for p in self.neighbors[name]['footprint'].polygon.points:
                        p_x = p.x
                        p_y = p.y
                        p_rotated = xform_foot((p_x, p_y))
                        p_x = STATIC_SCALE * p_rotated[0] + pos_rel[0]
                        p_y = STATIC_SCALE * p_rotated[1] + pos_rel[1]
                        cloud_points.append((p_x, p_y, Z_HEIGHT))
                        for _ in range(5):
                            scale = random.uniform(0.98, 1.02)
                            cloud_points.append((scale*p_x, scale*p_y, Z_HEIGHT))
                else:
                    if self.neighbors[name]['stationary']:
                        self.neighbors[name]['stationary'] = False
                        change = True

            if change or force_clear:
                self.clearing_laser_scan.header.stamp = self.me.header.stamp
                self.clearing_laser_pub.publish(self.clearing_laser_scan)

            static_robots_cloud = pcl2.create_cloud_xyz32(self.cloud_header, cloud_points)
            static_robots_cloud.header.stamp = self.me.header.stamp
            self.point_cloud_pub.publish(static_robots_cloud)

    def predict_pose(self, robot, time):
        time_delta = (time - robot['last_seen']).to_sec()
        pose = Pose()

        cur_pose = robot['position'].pose

        r, p, cur_theta = tf.transformations.euler_from_quaternion(quat_array_from_msg(cur_pose.orientation))
        v_x = robot['twist'].twist.linear.x
        v_y = robot['twist'].twist.linear.y
        v_th = robot['twist'].twist.angular.z
        delta_theta = v_th * time_delta
        q = tf.transformations.quaternion_from_euler(r, p, cur_theta + delta_theta)
        pose.orientation = Quaternion(*q)

        if False:
            if robot['holo_robot']:
                pose.position.x = cur_pose.position.x + v_x * time_delta
                pose.position.y = cur_pose.position.y + v_y * time_delta
            else:
                pose.position.x = cur_pose.position.x + v_x * np.cos(cur_theta + delta_theta / 2.)
                pose.position.y = cur_pose.position.y + v_x * np.sin(cur_theta + delta_theta / 2.)
        else:
            pose.position.x = cur_pose.position.x
            pose.position.y = cur_pose.position.y

        return pose


if __name__ == '__main__':
    rospy.init_node("position_share_controller")

    RATE = rospy.get_param('~rate', 10)
    global_frame = rospy.get_param('~global_frame', '/map')
    last_seen_threshold = rospy.get_param('~last_seen_threshold', 2.)
    Z_HEIGHT = 0.
    STATIC_SCALE = rospy.get_param("~static_scale", 0.95)

    controller = PositionShareController()
    rate = rospy.Rate(10)
    force_clear = False
    while not rospy.is_shutdown():
        controller.publish_static_robots(force_clear=force_clear)
        force_clear = not force_clear
        rate.sleep()
