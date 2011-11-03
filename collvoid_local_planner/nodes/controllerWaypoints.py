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


class controllerWaypoints():

    def __init__(self):
        self.initialize()
        

    def initialize(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
       
    
        self.pub_goal = rospy.Publisher("move_base/goal", MoveBaseActionGoal)
     
        self.init_guess_srv = rospy.ServiceProxy("init_guess_pub", Empty)

#        self.pub_commands_robot = rospy.Publisher("/commands_robot", String)
#        self.sub_common_position = rospy.Subscriber("/position_share", PoseTwistWithCovariance, self.cb_common_positions)
        self.sub_commands_robot = rospy.Subscriber("/commands_robot", String, self.cb_commands_robot)

        self.name = rospy.get_namespace()
        if (self.name == "/"):
            self.name = gethostname()

        rospy.loginfo("Name: %s",self.name)
        self.goal = rospy.get_param("/%s/goal"%self.name)
        self.cur_goal_msg = self.return_cur_goal()
        self.circle = False

        rospy.sleep(2.0)



    def cb_common_positions(self,msg):
        if self.stopped:
            return
        #       rospy.loginfo("%s"%rospy.get_master())
        if msg.id == self.hostname:
           
            if dist(msg.pose.pose, self.cur_goal_msg.pose) < THRESHOLD:
                rospy.loginfo("Reached goal, sending new goal")
                self.cur_goal += 1
                if (self.cur_goal == self.num_goals):
                    self.cur_goal = 0
                self.cur_goal_msg = self.return_cur_goal()
                self.pub_goal.publish(self.cur_goal_msg)
                if self.circle:
                    str = "Start"
                    self.pub_commands_robot.publish(String(str))



            return
        else:
            return
    
    def return_cur_goal(self):
        goal = MoveBaseGoal()
        #rospy.loginfo("goal: %s"%str(goal))
        
        goal.target_pose.pose.position.x = self.goal["x"]
        goal.target_pose.pose.position.y = self.goal["y"]
        q = tf.transformations.quaternion_from_euler(0,0, self.goal["ang"], axes='sxyz')
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        
#        goal.target_pose.header.stamp = rospy.get_time()
        goal.target_pose.header.frame_id = "/map"
        return goal


        


    def cb_commands_robot(self,msg):
        if msg.data == "all WP On" or msg.data == "%s WP On"%self.name:
            rospy.loginfo("I am ON")
            self.stopped = False

        if msg.data == "all WP Off" or msg.data == "%s WP Off"%self.name:
            self.stopped = True
            rospy.loginfo("I am Off")

        if (msg.data == "all Start"):
            self.client.send_goal(self.cur_goal_msg)

        if (msg.data == "all init Guess"):
            self.init_guess_srv()
        
        if (msg.data == "all Stop"):
            self.client.cancel_all_goals()
       
   

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
