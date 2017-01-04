#!/usr/bin/env python
import rospy
import actionlib
import math
import random
from std_msgs.msg import String
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from socket import gethostname
from collvoid_msgs.msg import PoseTwistWithCovariance
from move_base_msgs.msg import *

import tf.transformations

THRESHOLD = 0.22


def dist(a, b):
    return math.sqrt(math.pow(a.position.x - b.position.x, 2) + math.pow(a.position.y - b.position.y, 2))


class ControllerRobots(object):
    delayed_goal = None
    ground_truth = None

    def __init__(self):
        self.stopped = False

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        # self.client.wait_for_server()

        self.circling = False
        # self.init_guess_srv = rospy.ServiceProxy("init_guess_pub", InitGuess)

        self.hostname = rospy.get_namespace()
        self.noise_std = rospy.get_param("/noise_std", 0.0)
        self.covariance = rospy.get_param("/covariance", 0.005)

        if self.hostname == "/":
            self.hostname = gethostname()
            self.goals = rospy.get_param("/%s/goals" % self.hostname, [])
        else:
            self.goals = rospy.get_param("%sgoals" % self.hostname, [])
        self.hostname = self.hostname.replace('/', '')

        if len(self.goals) > 0:
            rospy.loginfo("goals: %s" % str(self.goals))
            self.cur_goal = 0
            self.num_goals = len(self.goals)
            self.cur_goal_msg = self.return_cur_goal()

        rospy.loginfo("Name: %s", self.hostname)
        self.pub_init_guess = rospy.Publisher("initialpose", PoseWithCovarianceStamped, queue_size=1)
        self.pub_commands_robot = rospy.Publisher("/commands_robot", String, queue_size=1)

        self.sub_commands_robot = rospy.Subscriber("/commands_robot", String, self.cb_commands_robot)
        self.sub_position_share = rospy.Subscriber("/position_share", PoseTwistWithCovariance, self.cb_common_positions)

        self.sub_goal = rospy.Subscriber("delayed_goal", PoseStamped, self.cb_delayed_goal)
        self.sub_ground_truth = rospy.Subscriber("base_pose_ground_truth", Odometry, self.cb_ground_truth)

        self.reset_srv = rospy.ServiceProxy('move_base/clear_local_costmap', Empty)

    def return_cur_goal(self):
        goal = MoveBaseGoal()
        goal.target_pose.pose.position.x = self.goals[self.cur_goal]["x"]
        goal.target_pose.pose.position.y = self.goals[self.cur_goal]["y"]
        q = tf.transformations.quaternion_from_euler(0, 0, self.goals[self.cur_goal]["ang"], axes='sxyz')
        goal.target_pose.pose.orientation = Quaternion(*q)

        goal.target_pose.header.frame_id = "/map"
        return goal

    def cb_common_positions(self, msg):
        if self.stopped or not self.circling:
            return  # rospy.loginfo("%s"%rospy.get_master())
        if msg.robot_id == self.hostname:
            if dist(msg.pose.pose, self.cur_goal_msg.target_pose.pose) < THRESHOLD:
                rospy.loginfo("Reached goal, sending new goal")
                self.cur_goal += 1
                if self.cur_goal == self.num_goals:
                    self.cur_goal = 0
                self.cur_goal_msg = self.return_cur_goal()
                str = "%s Start" % self.hostname
                self.pub_commands_robot.publish(String(str))

    def cb_delayed_goal(self, msg):
        self.delayed_goal = MoveBaseGoal()
        self.delayed_goal.target_pose.pose.position = msg.pose.position
        self.delayed_goal.target_pose.pose.orientation = msg.pose.orientation
        self.delayed_goal.target_pose.header = msg.header
        print str(self.delayed_goal)

    def cb_ground_truth(self, msg):
        self.ground_truth = PoseWithCovarianceStamped()
        self.ground_truth.header.frame_id = "/map"
        self.ground_truth.pose.pose = msg.pose.pose

        # self.ground_truth.pose.pose.position.x = -msg.pose.pose.position.y
        # self.ground_truth.pose.pose.position.y = msg.pose.pose.position.x
        # q = msg_to_quaternion(msg.pose.pose.orientation)
        # rpy = list(tf.transformations.euler_from_quaternion(q))
        # yaw = rpy[2] + math.pi / 2.0
        # q = tf.transformations.quaternion_from_euler(0,0,yaw, axes='sxyz')
        # self.ground_truth.pose.pose.orientation.x = q[0]
        # self.ground_truth.pose.pose.orientation.y = q[1]
        # self.ground_truth.pose.pose.orientation.z = q[2]
        # self.ground_truth.pose.pose.orientation.w = q[3]

    def publish_init_guess(self, noise_cov, noise_std):
        if self.ground_truth is not None:
            self.ground_truth.pose.pose.position.x += random.gauss(0, noise_std)
            self.ground_truth.pose.pose.position.y += random.gauss(0, noise_std)

            self.ground_truth.header.stamp = rospy.Time.now()
            self.ground_truth.pose.covariance[0] = noise_cov
            self.ground_truth.pose.covariance[7] = noise_cov
            self.ground_truth.pose.covariance[35] = noise_cov / 4.0
            self.pub_init_guess.publish(self.ground_truth)

    def cb_commands_robot(self, msg):
        # print msg.data
        if "all" not in msg.data and self.hostname not in msg.data:
            return

        if "WP Change" in msg.data:
            self.stopped = not self.stopped

        if self.stopped:
            rospy.loginfo("I am stopped %s", self.hostname)
            return

        if "init Guess" in msg.data:
            self.publish_init_guess(self.covariance, self.noise_std)
            try:
                self.reset_srv()
                self.reset_srv()
            except rospy.ServiceException as e:
                rospy.logwarn(e)
            rospy.sleep(0.2)
            self.publish_init_guess(self.covariance, self.noise_std)

        if "Restart" in msg.data:
            try:
                self.reset_srv()
                self.reset_srv()
            except rospy.ServiceException as e:
                rospy.logwarn(e)

        if "Stop" in msg.data:
            self.client.cancel_all_goals()

        if "Start" in msg.data:
            self.client.send_goal(self.cur_goal_msg)

        if "circle" in msg.data:
            self.circling = not self.circling
            rospy.loginfo("Set circling to %s", str(self.circling))
        if "next Goal" in msg.data:
            self.cur_goal += 1
            if self.cur_goal == self.num_goals:
                self.cur_goal = 0
            self.cur_goal_msg = self.return_cur_goal()
            self.client.send_goal(self.cur_goal_msg)
            rospy.loginfo("Send new Goal")

        if "send delayed Goal" in msg.data:
            self.client.send_goal(self.delayed_goal)


if __name__ == '__main__':
    rospy.init_node('controller_robots')
    controller_waypoints = ControllerRobots()

    r = rospy.Rate(20)
    while not rospy.is_shutdown():
        r.sleep()
    rospy.delete_param("/move_base/")
