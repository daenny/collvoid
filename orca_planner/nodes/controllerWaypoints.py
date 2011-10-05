#!/usr/bin/env python
import roslib; roslib.load_manifest('orca_planner')
import rospy
import commands
import string
import sys
import math
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped,PoseStamped
from socket import gethostname
from orca_planner.msg import PositionShare

import tf

THRESHOLD = 0.1

def dist(a, b):
    return math.sqrt(math.pow(a.position.x - b.position.x, 2) + math.pow(a.position.y - b.position.y, 2))


class controllerWaypoints():

    def __init__(self):
        self.initialize()
        

    def initialize(self):
        self.pub_goal = rospy.Publisher("/goal", PoseStamped)

        self.sub_common_position = rospy.Subscriber("/commonPositions", PositionShare, self.cb_common_positions)
        self.sub_commands_robot = rospy.Subscriber("/commandsRobot", String, self.cb_commands_robot)
        self.hostname = gethostname()
#        self.hostname = "turtlebot4"
        rospy.logerr("Hostname: %s",self.hostname)
        self.goals = rospy.get_param("/%s/goals"%self.hostname)
        rospy.loginfo("goals: %s"%str(self.goals))
        self.cur_goal = 0
        self.num_goals = len(self.goals["x"])
        rospy.loginfo("num Goals %d"%self.num_goals)
#        rospy.loginfo("Cur Goal %s"%str(self.return_cur_goal()))
        self.cur_goal_msg = self.return_cur_goal()


    def cb_common_positions(self,msg):
        #       rospy.loginfo("%s"%rospy.get_master())
        if msg.id == self.hostname:
           
            if dist(msg.pose.pose, self.cur_goal_msg.pose) < THRESHOLD:
                rospy.loginfo("Reached goal, sending new goal")
                self.cur_goal += 1
                if (self.cur_goal == self.num_goals):
                    self.cur_goal = 0
                self.cur_goal_msg = self.return_cur_goal()
                self.pub_goal.publish(self.cur_goal_msg)
            
            return
        else:
            return
    
    def return_cur_goal(self):
        goal = PoseStamped()
        goal.pose.position.x = self.goals["x"][self.cur_goal]
        goal.pose.position.y = self.goals["y"][self.cur_goal]
        return goal
        


    def cb_commands_robot(self,msg):
        if (msg.data == "Start"):
            self.pub_goal.publish(self.return_cur_goal())
        
        if msg.data == "New Goal":
            self.cur_goal += 1
            if (self.cur_goal == self.num_goals):
                self.cur_goal = 0
            self.cur_goal_msg = self.return_cur_goal()
            self.pub_goal.publish(self.cur_goal_msg)

        if msg.data == "Circle On":
            THRESHOLD = 0.2
            
        if msg.data == "Circle Off":
            THRESHOLD = 0.1
            

#-- circle between
#-- add point



if __name__ == '__main__':
#    if not (len(sys.argv) == 2):
#        print "usage: controllerWaypoints.py <hostname>"
#        sys.exit(-1)


 #   hostname = argv = sys.argv[1]

        
    rospy.init_node('controller_waypoints')
    controller_waypoints = controllerWaypoints()
    rospy.spin()
#    r = rospy.Rate(100)
#    while not rospy.is_shutdown():
#        r.sleep()
