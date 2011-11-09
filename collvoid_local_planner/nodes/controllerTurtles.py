#!/usr/bin/env python
import roslib; roslib.load_manifest('collvoid_local_planner')
import rospy
import commands
import string
import sys
import math
import actionlib
from std_srvs.srv import Empty
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped,PoseStamped
from socket import gethostname
from collvoid_msgs.msg import PoseTwistWithCovariance
from move_base_msgs.msg import *

import tf

THRESHOLD = 0.1


def dist(a, b):
    return math.sqrt(math.pow(a.position.x - b.position.x, 2) + math.pow(a.position.y - b.position.y, 2))


class ControllerTurtles():

    def __init__(self):
        self.initialize()
        

    def initialize(self):
        self.stopped = False

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
       
    
        self.pub_goal = rospy.Publisher("move_base/goal", MoveBaseActionGoal)
     
        self.init_guess_srv = rospy.ServiceProxy("init_guess_pub", Empty)

#        self.pub_commands_robot = rospy.Publisher("/commands_robot", String)
#        self.sub_common_position = rospy.Subscriber("/position_share", PoseTwistWithCovariance, self.cb_common_positions)
        self.sub_commands_robot = rospy.Subscriber("/commands_robot", String, self.cb_commands_robot)

        self.hostname = rospy.get_namespace()
        if (self.hostname == "/"):
            self.hostname = gethostname()

        self.goals = rospy.get_param("/%s/goals"%self.hostname)
        rospy.loginfo("goals: %s"%str(self.goals))
        self.cur_goal = 0
        self.num_goals = len(self.goals["x"])
#        rospy.loginfo("num Goals %d"%self.num_goals)
        rospy.loginfo("Cur Goal %s"%str(self.return_cur_goal()))
        self.cur_goal_msg = self.return_cur_goal()
        self.circle = False
        rospy.loginfo("Name: %s",self.hostname)



    def cb_common_positions(self,msg):
        if self.stopped:
            return
        #       rospy.loginfo("%s"%rospy.get_master())
        if msg.id == self.hostname:
           
            if dist(msg.pose.pose, self.cur_goal_msg.target_pose.pose) < THRESHOLD:
                rospy.loginfo("Reached goal, sending new goal")
                self.cur_goal += 1
                if (self.cur_goal == self.num_goals):
                    self.cur_goal = 0
                self.cur_goal_msg = self.return_cur_goal()
                if self.circle:
                    str = "%s Start"%self.hostname
                    self.pub_commands_robot.publish(String(str))
            return
        else:
            return
    
    def return_cur_goal(self):
        goal = MoveBaseGoal()
        #rospy.loginfo("goal: %s"%str(goal))
        
        goal.target_pose.pose.position.x = self.goals["x"][self.cur_goal]
        goal.target_pose.pose.position.y = self.goals["y"][self.cur_goal]
        q = tf.transformations.quaternion_from_euler(0,0, self.goals["ang"][self.cur_goal], axes='sxyz')
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        
#        goal.target_pose.header.stamp = rospy.get_time()
        goal.target_pose.header.frame_id = "/map"
        return goal


        


    def cb_commands_robot(self,msg):
        if msg.data == "all WP On" or msg.data == "%s WP On"%self.hostname:
            rospy.loginfo("I am ON")
            self.stopped = False

        if msg.data == "all WP Off" or msg.data == "%s WP Off"%self.hostname:
            self.stopped = True
            rospy.loginfo("I am Off")

        if (msg.data == "all init Guess"):
            self.init_guess_srv()
        
        if msg.data == "all Stop" or msg.data == "%s Stop"%self.hostname:
            self.client.cancel_all_goals()
          
        if self.stopped:
            return

        if msg.data == "all Start" or msg.data == "%s Start"%self.hostname:
            self.client.send_goal(self.cur_goal_msg)
        
        if msg.data == "all next Goal" or msg.data == "%s next Goal"%self.hostname:
            self.cur_goal += 1
            if (self.cur_goal == self.num_goals):
                self.cur_goal = 0
            self.cur_goal_msg = self.return_cur_goal()
            self.pub_goal.publish(self.cur_goal_msg)
            rospy.loginfo("Send new Goal")            

        if msg.data == "all Circle On" or msg.data == "%s Circle On"%self.hostname:
            rospy.loginfo("I am on Circlemode")
            self.circle = True
            
        if msg.data == "all Circle Off" or msg.data == "%s Circle Off"%self.hostname:
            self.circle = False


if __name__ == '__main__':
#    if not (len(sys.argv) == 2):
#        print "usage: controllerWaypoints.py <hostname>"
#        sys.exit(-1)


 #   hostname = argv = sys.argv[1]

        
    rospy.init_node('controller_turtles')
    controller_waypoints = ControllerTurtles()
    rospy.spin()
#    r = rospy.Rate(100)
#    while not rospy.is_shutdown():
#        r.sleep()
