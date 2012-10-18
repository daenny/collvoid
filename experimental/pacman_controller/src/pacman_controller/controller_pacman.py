#!/usr/bin/env python
import roslib; roslib.load_manifest('pacman_controller')
import rospy
import commands
import string
import sys
import math
import actionlib
import random
from collvoid_local_planner.srv import InitGuess
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped,PoseStamped
from socket import gethostname
from collvoid_msgs.msg import PoseTwistWithCovariance
from move_base_msgs.msg import *

import tf

from pacman_controller import game_engine as GE


class ControllerPacman():

    def __init__(self):
        self.initialize()
        

    def initialize(self):
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()
        self.init_guess_srv = rospy.ServiceProxy("init_guess_pub", InitGuess)

        self.ghostname = rospy.get_param("~ghostname")
        self.home = rospy.get_param("/%s/home"%self.ghostname)
        
        
        rospy.loginfo("I am: %s home %d",self.ghostname, self.home)
        self.gameEngine = GE.GameEngine()
        self.state = GE.State.STOPPED
        self.dead = False
        self.current_wp = None


        #self.pub_goal = rospy.Publisher("move_base/goal", MoveBaseActionGoal)
        self.sub_state = rospy.Subscriber("/state", String, self.cb_state)
        self.sub_commands_robot = rospy.Subscriber("/commands_robot", String, self.cb_commands_robot)
  
        
   
    def update(self):
        if self.state == GE.State.SETUP:
            return
        new_wp = None
        if (self.state == GE.State.STOPPED or self.state == GE.State.PAUSED):
            self.move_base_client.cancel_all_goals()
            self.current_wp = None
            return
        if (self.state in [GE.State.INIT, GE.State.GAME_OVER, GE.State.WON] or self.dead):
            new_wp = self.return_home();
        if not new_wp == None and not (new_wp == self.current_wp):
            self.current_wp = new_wp
            self.move_base_client.cancel_all_goals()
            self.send_goal(self.current_wp)
        
    def send_goal(self,wp):
        goal = MoveBaseGoal()
        goal.target_pose.pose.position.x = self.gameEngine.points[wp]["x"]
        goal.target_pose.pose.position.y = self.gameEngine.points[wp]["y"]
        #q = tf.transformations.quaternion_from_euler(0,0, self.home["ang"], axes='sxyz')
        goal.target_pose.pose.orientation.x = 0
        goal.target_pose.pose.orientation.y = 0
        goal.target_pose.pose.orientation.z = 0
        goal.target_pose.pose.orientation.w = 1.0
        
#        home.target_pose.header.stamp = rospy.get_time()
        goal.target_pose.header.frame_id = "/map"
       
        self.move_base_client.send_goal(goal)
        return

    def cb_state(self,msg):
        if self.state == GE.State.INIT and int(msg.data) == GE.State.RUNNING:
            self.current_wp = None
            self.move_base_client.cancel_all_goals()
        if self.state in [GE.State.RUNNING, GE.State.FLEEING] and int(msg.data) in [GE.State.STOPPED, GE.State.PAUSED, GE.State.WON, GE.State.GAME_OVER]:
            self.move_base_client.cancel_all_goals()
            self.current_wp = None
        if not self.state == GE.State.SETUP and int(msg.data) == GE.State.SETUP:
            self.move_base_client.cancel_all_goals()
 

        self.state = int(msg.data)

       
    def return_home(self):
        if GE.dist(self.gameEngine.points[self.home], self.gameEngine.pacman) < self.gameEngine.pacman['radius'] + GE.THRESHOLD:
            self.dead = False
        my_position = self.gameEngine.pacman
        my_wp = self.gameEngine.findClosestMapPoint(my_position)
       
        path = self.gameEngine.find_shortest_path(my_wp,self.gameEngine.points[self.home])
        index = 1
        while index < len(path)-1 and GE.dist(path[index], self.gameEngine.pacman) < self.gameEngine.pacman['radius'] + 1.5 * GE.THRESHOLD :
            index += 1;

        return path[index]['id']

    def cb_commands_robot(self,msg):
        if (msg.data == "init Guess"):
            self.init_guess_srv(0.1)
  
if __name__ == '__main__':
    #if not (len(sys.argv) == 2):
    #    print "usage: controllerWaypoints.py <ghostname>"
    #    sys.exit(-1)

    #ghostname = sys.argv[1]
    rospy.init_node('controller_pacman')
    controller_pacman = ControllerPacman()
    while not rospy.is_shutdown():
        controller_pacman.update()
        rospy.sleep(0.1)


