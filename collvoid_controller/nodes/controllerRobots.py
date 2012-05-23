#!/usr/bin/env python
import roslib; roslib.load_manifest('collvoid_controller')
import rospy
import string
import actionlib
import math
#from collvoid_local_planner.srv import InitGuess
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped,PoseStamped
from nav_msgs.msg import Odometry
from socket import gethostname
from collvoid_msgs.msg import PoseTwistWithCovariance
from move_base_msgs.msg import *

import tf

class ControllerRobots():

    def __init__(self):
        self.initialize()
        

    def initialize(self):
        self.stopped = False

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
       
        #self.init_guess_srv = rospy.ServiceProxy("init_guess_pub", InitGuess)
        self.sub_commands_robot = rospy.Subscriber("/commands_robot", String, self.cb_commands_robot)

        self.sub_goal = rospy.Subscriber("/goal", PoseStamped, self.cb_goal)
        self.sub_ground_truth = rospy.Subscriber("base_pose_ground_truth", Odometry, self.cb_ground_truth)
       
        self.pub_init_guess = rospy.Publisher("initialpose", PoseWithCovarianceStamped)

        self.hostname = rospy.get_namespace()

        if (self.hostname == "/"):
            self.hostname = gethostname()
            self.goals = rospy.get_param("/%s/goals"%self.hostname,[])
        else:
            self.goals = rospy.get_param("%sgoals"%self.hostname,[])
        if len(self.goals)>0:
            rospy.loginfo("goals: %s"%str(self.goals))
            self.cur_goal = 0
            self.num_goals = len(self.goals["x"])
            self.cur_goal_msg = self.return_cur_goal()
        rospy.loginfo("Name: %s",self.hostname)



    def return_cur_goal(self):
        goal = MoveBaseGoal()
        goal.target_pose.pose.position.x = self.goals["x"][self.cur_goal]
        goal.target_pose.pose.position.y = self.goals["y"][self.cur_goal]
        q = tf.transformations.quaternion_from_euler(0,0, self.goals["ang"][self.cur_goal], axes='sxyz')
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        
        goal.target_pose.header.frame_id = "/map"
        return goal


    def cb_goal(self,msg):
        self.sent_goal = MoveBaseGoal()
        self.sent_goal.target_pose.pose.position = msg.pose.position
        self.sent_goal.target_pose.pose.orientation = msg.pose.orientation
        self.sent_goal.target_pose.header = msg.header;
        print str(self.sent_goal)

    def cb_ground_truth(self, msg):
        self.ground_truth = PoseWithCovarianceStamped()
        #print str(self.ground_truth)
        self.ground_truth.header.frame_id = msg.header.frame_id
        self.ground_truth.pose.pose.position.x = -msg.pose.pose.position.y
        self.ground_truth.pose.pose.position.y = msg.pose.pose.position.x
        q = msg_to_quaternion(msg.pose.pose.orientation)
        rpy = list(tf.transformations.euler_from_quaternion(q))
        yaw = rpy[2] + math.pi / 2.0
        q = tf.transformations.quaternion_from_euler(0,0,yaw, axes='sxyz')
        self.ground_truth.pose.pose.orientation.x = q[0]
        self.ground_truth.pose.pose.orientation.y = q[1]
        self.ground_truth.pose.pose.orientation.z = q[2]
        self.ground_truth.pose.pose.orientation.w = q[3]
        

    def publish_init_guess(self, noise_cov):
        if not (self.ground_truth == None):
            self.ground_truth.header.stamp = rospy.Time.now()
            self.ground_truth.pose.covariance[0] = noise_cov
            self.ground_truth.pose.covariance[7] = noise_cov
            self.ground_truth.pose.covariance[35] = noise_cov / 3.0
            self.pub_init_guess.publish(self.ground_truth)
        
    def cb_commands_robot(self,msg):
        if (msg.data == "all init Guess"):
            self.publish_init_guess(0.005)
        
        if msg.data == "all Stop" or msg.data == "%s Stop"%self.hostname:
            self.client.cancel_all_goals()
          
        if msg.data == "all Start" or msg.data == "%s Start"%self.hostname:
            self.client.send_goal(self.cur_goal_msg)
        
        if msg.data == "all next Goal" or msg.data == "%s next Goal"%self.hostname:
            self.cur_goal += 1
            if (self.cur_goal == self.num_goals):
                self.cur_goal = 0
            self.cur_goal_msg = self.return_cur_goal()
            self.client.send_goal(self.cur_goal_msg)

            rospy.loginfo("Send new Goal")

        if msg.data == "all send delayed Goal" or msg.data == "%s send delayed Goal"%self.hostname:
            self.client.send_goal(self.sent_goal)
        
def msg_to_quaternion(msg):
    return [msg.x, msg.y, msg.z, msg.w]


if __name__ == '__main__':
    rospy.init_node('controller_robots')
    controller_waypoints = ControllerRobots()

    r = rospy.Rate(20)
    while not rospy.is_shutdown():
        r.sleep()
    rospy.delete_param("/move_base/")
