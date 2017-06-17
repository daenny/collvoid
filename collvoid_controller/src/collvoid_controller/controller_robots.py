#!/usr/bin/env python
import rospy
import actionlib
import math
import random

from std_msgs.msg import String
from std_srvs.srv import Empty, Trigger
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Quaternion, Twist
from nav_msgs.msg import Odometry
from socket import gethostname
from collvoid_msgs.msg import PoseTwistWithCovariance
from move_base_msgs.msg import *

import tf.transformations

MAX_ZERO_COUNT = 20
THRESHOLD = 0.15


def dist(a, b):
    return math.sqrt(math.pow(a.position.x - b.position.x, 2) + math.pow(a.position.y - b.position.y, 2))


class ControllerRobots(object):
    delayed_goal = None
    ground_truth = None
    zero_count = 0

    def __init__(self):
        self.stopped = True

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        # self.client.wait_for_server()

        self.circling = False
        # self.init_guess_srv = rospy.ServiceProxy("init_guess_pub", InitGuess)
        self.stalled = False
        self.goal_reached = True
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
            self.cur_goal = -1
            self.num_goals = len(self.goals)
            self.cur_goal_msg = None

        rospy.loginfo("Name: %s", self.hostname)
        self.pub_init_guess = rospy.Publisher("initialpose", PoseWithCovarianceStamped, queue_size=1)
        self.pub_cmd_vel = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.cur_goal_pub = rospy.Publisher("current_goal", PoseStamped, queue_size=1)

        self.pub_commands_robot = rospy.Publisher("/commands_robot", String, queue_size=1)

        self.sub_commands_robot = rospy.Subscriber("/commands_robot", String, self.cb_commands_robot)
        self.sub_position_share = rospy.Subscriber("/position_share", PoseTwistWithCovariance, self.cb_common_positions, queue_size=1)

        self.sub_goal = rospy.Subscriber("delayed_goal", PoseStamped, self.cb_delayed_goal)
        self.sub_ground_truth = rospy.Subscriber("base_pose_ground_truth", Odometry, self.cb_ground_truth, queue_size=1)

        use_sim = rospy.get_param("/use_sim_time", default=False)
        if use_sim:
            from stage_ros.msg import Stall
            self.sub_stall = rospy.Subscriber("stall", Stall, self.cb_stall)

        self.reset_srv = rospy.ServiceProxy('move_base/clear_local_costmap', Empty)

        self.global_reset_srv = rospy.ServiceProxy('move_base/clear_costmaps', Empty)
        rospy.Service('is_done', Trigger, self.cb_is_done)

    def cb_is_done(self, req):
        if self.goal_reached or self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            return {'success': True}
        return {'success': False}

    def return_cur_goal(self):
        goal = MoveBaseGoal()
        goal.target_pose.pose.position.x = self.goals[self.cur_goal]["x"]
        goal.target_pose.pose.position.y = self.goals[self.cur_goal]["y"]
        q = tf.transformations.quaternion_from_euler(0, 0, self.goals[self.cur_goal]["ang"], axes='sxyz')
        goal.target_pose.pose.orientation = Quaternion(*q)

        goal.target_pose.header.frame_id = "/map"
        return goal

    def cb_stall(self, msg):
        self.stalled = msg.stall

    def cb_common_positions(self, msg):
        if self.stopped or self.cur_goal_msg is None:
            return  # rospy.loginfo("%s"%rospy.get_master())
        if msg.robot_id == self.hostname:
            if dist(msg.pose.pose, self.cur_goal_msg.target_pose.pose) < THRESHOLD:
                if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED and not self.circling:
                    rospy.loginfo("Reached last goal")
                    self.goal_reached = True
                    self.stopped = True

                #if self.cur_goal == self.num_goals-1 and not self.circling:
                #    if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                #        rospy.loginfo("Reached last goal")
                #        rospy.logerr("I AM DONE")
                #        self.stopped = True
                #    return
                #else:
                if self.circling:
                    self.cur_goal += 1
                    if self.circling and self.cur_goal == self.num_goals:
                        self.cur_goal = 0
                    rospy.loginfo("sending new goal %d/%d", self.cur_goal, self.num_goals)
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

        if self.stopped:
            return
        if msg.twist.twist.linear.x == 0 and msg.twist.twist.linear.y == 0 and msg.twist.twist.angular.z == 0:
            self.zero_count += 1
            if self.zero_count > MAX_ZERO_COUNT:
                if self.stalled:
                    twist = Twist()
                    twist.linear.x = -0.05
                    self.pub_cmd_vel.publish(twist)
                else:
                    twist = Twist()
                    twist.linear.x = 0.05
                    self.pub_cmd_vel.publish(twist)
                #self.client.cancel_all_goals()
                #self.client.send_goal(self.cur_goal_msg)
                str = "%s Start" % self.hostname
                self.pub_commands_robot.publish(String(str))
        else:
            self.zero_count = 0

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

        if "Start" in msg.data:
            try:
                self.global_reset_srv()
                self.reset_srv()
            except:
                pass
            if self.cur_goal == -1:
                self.cur_goal = 0
            self.cur_goal_msg = self.return_cur_goal()
            rospy.loginfo("sending new goal %d/%d", self.cur_goal, self.num_goals)
            self.stopped = False
            self.goal_reached = False
            self.client.send_goal(self.cur_goal_msg)
            self.cur_goal_pub.publish(self.cur_goal_msg.target_pose)

        if "init Guess" in msg.data:
            self.publish_init_guess(self.covariance, self.noise_std)
            try:
                self.global_reset_srv()
                self.global_reset_srv()
                self.reset_srv()
                self.reset_srv()
            except rospy.ServiceException as e:
                rospy.logwarn(e)
            rospy.sleep(0.2)
            self.publish_init_guess(self.covariance, self.noise_std)

        if "Restart" in msg.data:
            self.stopped = True
            self.client.cancel_all_goals()
            try:
                self.global_reset_srv()
                self.global_reset_srv()
                self.reset_srv()
                self.reset_srv()
            except rospy.ServiceException as e:
                rospy.logwarn(e)

        if "Stop" in msg.data:
            self.stopped = True
            self.client.cancel_all_goals()

        if "circle" in msg.data:
            self.circling = not self.circling
            rospy.loginfo("Set circling to %s", str(self.circling))

        if "send Goal num" in msg.data:
            self.cur_goal = int(msg.data.split(" ")[-1])
            try:
                self.global_reset_srv()
                self.reset_srv()
            except:
                pass
            self.goal_reached = False
            rospy.loginfo("sending new goal %d/%d", self.cur_goal, self.num_goals)
            self.cur_goal_msg = self.return_cur_goal()
            self.stopped = False
            self.client.send_goal(self.cur_goal_msg)
            self.cur_goal_pub.publish(self.cur_goal_msg.target_pose)
            rospy.loginfo("Send new Goal")

        if "next Goal" in msg.data:
            self.cur_goal += 1
            if self.cur_goal == self.num_goals:
                self.cur_goal = 0
            try:
                self.global_reset_srv()
                self.reset_srv()
            except:
                pass
            self.goal_reached = False
            rospy.loginfo("sending new goal %d/%d", self.cur_goal, self.num_goals)
            self.cur_goal_msg = self.return_cur_goal()
            self.stopped = False
            self.client.send_goal(self.cur_goal_msg)
            self.cur_goal_pub.publish(self.cur_goal_msg.target_pose)
            rospy.loginfo("Send new Goal")

        if "send delayed Goal" in msg.data:
            if self.delayed_goal is not None:
                self.stopped = False
                self.goal_reached = False
                self.client.send_goal(self.delayed_goal)


if __name__ == '__main__':
    rospy.init_node('controller_robots')
    controller_waypoints = ControllerRobots()

    r = rospy.Rate(20)
    while not rospy.is_shutdown():
        r.sleep()
    rospy.delete_param("/move_base/")
