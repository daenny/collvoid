#!/usr/bin/env python
__author__ = 'danielclaes'
import rospy
from geometry_msgs.msg import PolygonStamped, PointStamped, Point, PoseStamped, PoseWithCovariance, Pose, Quaternion
from collvoid_msgs.msg import PoseTwistWithCovariance
from collvoid_srvs.srv import GetNeighbors, GetNeighborsResponse
from socket import gethostname
from sensor_msgs.msg import LaserScan, PointCloud
import tf
import tf.transformations
import cv2
import cv2.cv
import numpy as np

RATE = rospy.get_param('~rate', 10)
global_frame = rospy.get_param('~global_frame', '/map')
last_seen_threshold = rospy.get_param('~last_seen_threshold', 2.)

class PositionShareController(object):
    def __init__(self):
        self.position_share_pub = rospy.Publisher('/position_share', PoseTwistWithCovariance, queue_size=1)
        # Set the name
        self.name = rospy.get_namespace()
        if self.name == "/":
            self.name = gethostname()
        else:
            self.name = self.name.replace('/', '')

        rospy.loginfo("Position Share started with name: %s", self.name)

        self.name = rospy.get_param('~name', self.name)
        self.neighbors = {}

        rospy.Subscriber('/position_share', PoseTwistWithCovariance, self.position_share_cb)
        rospy.Service('get_neighbors', GetNeighbors, self.get_neighbors_cb)

    def position_share_cb(self, msg):
        """
        :param msg:
        :type msg: PoseTwistWithCovariance
        """
        if msg.robot_id == self.name:
            return
        if msg.robot_id not in self.neighbors.keys():
            self.neighbors[msg.robot_id] = {}

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
        robot['last_seen'] = msg.header.stamp # or rospy.Time.now()

    def get_neighbors_cb(self, req):
        response = GetNeighborsResponse()
        time = rospy.Time.now()
        for name in self.neighbors.keys():
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

    def predict_pose(self, robot, time):
        time_delta = (time - robot['last_seen']).to_sec()
        pose = Pose()

        cur_pose = robot['position'].pose
        quat_array = lambda o: np.array([o.x, o.y, o.z, o.w])

        r, p, cur_theta = tf.transformations.euler_from_quaternion(quat_array(cur_pose.orientation))
        v_x = robot['twist'].twist.linear.x
        v_y = robot['twist'].twist.linear.y
        v_th = robot['twist'].twist.angular.z
        delta_theta = v_th * time_delta
        q = tf.transformations.quaternion_from_euler(r, p, cur_theta + delta_theta)
        pose.orientation = Quaternion(*q)

        if robot['holo_robot']:
            pose.position.x = cur_pose.position.x + v_x * time_delta
            pose.position.y = cur_pose.position.y + v_y * time_delta
        else:
            pose.position.x = cur_pose.position.x + v_x * np.cos(cur_theta + delta_theta/2.)
            pose.position.y = cur_pose.position.y + v_x * np.sin(cur_theta + delta_theta/2.)
        return pose

    def spin(self):
        rate = rospy.Rate(RATE)
        while rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node("position_share_controller")
    controller = PositionShareController()
    rospy.spin()
    # controller.spin()

