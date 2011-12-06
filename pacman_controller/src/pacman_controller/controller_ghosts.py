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


class ControllerGhosts():

    def __init__(self):
        self.initialize()
        

    def initialize(self):
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()
        self.init_guess_srv = rospy.ServiceProxy("init_guess_pub", InitGuess)

        #self.pub_goal = rospy.Publisher("move_base/goal", MoveBaseActionGoal)
  
        self.ghostname = rospy.get_param("~ghostname")
        self.home = rospy.get_param("/%s/home"%self.ghostname)
        
        
        rospy.loginfo("Ghost: %s home %d",self.ghostname, self.home)
        self.gameEngine = GE.GameEngine()
        self.state = GE.State.STOPPED
        self.dead = False
        self.headingToWp = False
        self.current_wp = None
        self.old_wp = self.gameEngine.points[self.home]
        self.sleepTime = rospy.get_param('/%s/sleep_time'%self.ghostname)
  

        self.sub_state = rospy.Subscriber("/state", String, self.cb_state)
        self.sub_commands_robot = rospy.Subscriber("/commands_robot", String, self.cb_commands_robot)

   
    def update(self):
        if not self.gameEngine.initialized or not self.ghostname in self.gameEngine.ghosts or self.state == GE.State.SETUP:
            return
        me = self.gameEngine.ghosts[self.ghostname]
                          
        if self.state == GE.State.RUNNING and self.startTime + self.sleepTime > rospy.get_time():
            return
 

        if (self.state == GE.State.STOPPED or self.state == GE.State.PAUSED):
            self.move_base_client.cancel_all_goals()
            self.current_wp = None
            self.headingToWp = False
            #self.old_wp = self.gameEngin.points[self.home]
            return
  
        if (self.state in [GE.State.INIT, GE.State.GAME_OVER] or self.dead):
            new_wp,end = self.return_home();
            self.headingToWp = False
        elif (self.state == GE.State.FLEEING):
            if GE.dist(self.gameEngine.pacman, me) < self.gameEngine.pacman['radius'] + me['radius'] + 1.5*GE.THRESHOLD:
                self.dead = True
                new_wp,end = self.return_home();
                self.headingToWp = False
            else:
                new_wp,end = self.newBehavior()
                  
        elif self.headingToWp and GE.dist(me, self.gameEngine.points[self.current_wp]) < me['radius'] + GE.THRESHOLD:
            self.headingToWp = False
            return
        elif self.headingToWp:
            return
        else:
            new_wp,end = self.newBehavior()
        
        if not (new_wp == self.current_wp):
            self.current_wp = new_wp
            self.move_base_client.cancel_all_goals()
            self.send_goal(self.current_wp)
        else:
            nearestWP = self.gameEngine.findClosestWP(me)
            if self.state in [GE.State.RUNNING, GE.State.FLEEING] and nearestWP == self.current_wp and self.current_wp == end and GE.dist(me, self.gameEngine.points[self.current_wp]) < me['radius']:
                self.headingToWp = True
                
                pm_wp = self.gameEngine.findClosestWP(self.gameEngine.pacman)
                if (end == pm_wp):
                    pm_mp = self.gameEngine.findClosestMapPoint(self.gameEngine.pacman)
                    if pm_mp['neighbors'][0] == pm_wp:
                        self.current_wp = pm_mp['neighbors'][1]
                    else:
                        self.current_wp = pm_mp['neighbors'][0]
                    
                else:
                    while self.current_wp == end:
                        print self.ghostname + str(self.current_wp)
                        self.current_wp = random.choice(self.gameEngine.points[nearestWP]['neighbors'])
               
                self.move_base_client.cancel_all_goals()
                self.send_goal(self.current_wp)

    def newBehavior(self):
        if (self.ghostname == "ghost_red"):
            return self.chase_pacman()
        if (self.ghostname == "ghost_pink"):
            return self.move_random(1.0)
        if (self.ghostname == "ghost_blue"):
            return self.move_random(0.2)
                    
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
            self.startTime = rospy.get_time()
        if not self.state == GE.State.SETUP and int(msg.data) == GE.State.SETUP:
            self.move_base_client.cancel_all_goals()

        self.state = int(msg.data)
        

    def flee(self):
        my_position = self.gameEngine.ghosts[self.ghostname]
        pm_position = self.gameEngine.pacman

        my_wp = self.gameEngine.findClosestMapPoint(my_position)
        pm_wp = self.gameEngine.findClosestWP(pm_position)
        path = self.gameEngine.find_shortest_path(my_wp, pm_wp)

        if not (my_wp['neighbors'][0] == path[1]['id']):
            return my_wp['neighbors'][0]
        else:
            return my_wp['neighbors'][1]

        
    def move_random(self, percent):
        me = self.gameEngine.ghosts[self.ghostname]
        
        if GE.dist(me, self.old_wp) < me['radius'] + GE.THRESHOLD:
            if random.random() <= percent and not self.old_wp['id'] == self.home:
                self.old_wp = random.choice(self.gameEngine.points)
            else:
                self.old_wp = self.gameEngine.points[self.gameEngine.findClosestWP(self.gameEngine.pacman)]
            
        my_wp = self.gameEngine.findClosestMapPoint(me)
        path = self.gameEngine.find_shortest_path(my_wp, self.old_wp)


        if len(path) == 1:
            return path[0]['id'], path[len(path)-1]['id']

        index = 1
        
        while index < len(path)-1 and GE.dist(path[index], me) < me['radius'] + GE.THRESHOLD :
            index += 1;

        while index < len(path)-1 and GE.dist(path[index], self.gameEngine.ghosts[self.ghostname]) < 2 * self.gameEngine.ghosts[self.ghostname]['radius'] + 2.0 * GE.THRESHOLD and self.gameEngine.positionBlocked(path[index], self.gameEngine.ghosts[self.ghostname]):
            index +=1;

 
        return path[index]['id'], path[len(path)-1]['id']

                
    def chase_pacman(self):
        my_position = self.gameEngine.ghosts[self.ghostname]
        pm_position = self.gameEngine.pacman
        
        my_wp = self.gameEngine.findClosestMapPoint(my_position)
        pm_wp = self.gameEngine.findClosestWP(pm_position)
        path = self.gameEngine.find_shortest_path(my_wp, self.gameEngine.points[pm_wp])
        
        if len(path) == 1:
            return path[0]['id'], path[len(path)-1]['id']
        
        index = 1

        while index < len(path)-1 and GE.dist(path[index], self.gameEngine.ghosts[self.ghostname]) < self.gameEngine.ghosts[self.ghostname]['radius'] + GE.THRESHOLD :
            index += 1;

 
        return path[index]['id'], path[len(path)-1]['id']


    def return_home(self):
        if GE.dist(self.gameEngine.points[self.home], self.gameEngine.ghosts[self.ghostname]) < self.gameEngine.ghosts[self.ghostname]['radius'] + GE.THRESHOLD:
            self.dead = False
        my_position = self.gameEngine.ghosts[self.ghostname]
        my_wp = self.gameEngine.findClosestMapPoint(my_position)
       
        path = self.gameEngine.find_shortest_path(my_wp,self.gameEngine.points[self.home])
        
        if len(path) == 1:
            return path[0]['id'], path[len(path)-1]['id']

        index = 1
        while index < len(path)-1 and GE.dist(path[index], self.gameEngine.ghosts[self.ghostname]) < self.gameEngine.ghosts[self.ghostname]['radius'] + 1.5 * GE.THRESHOLD :
            index += 1;
        
        while index < len(path)-1 and GE.dist(path[index], self.gameEngine.ghosts[self.ghostname]) < 2 * self.gameEngine.ghosts[self.ghostname]['radius'] + 2.0 * GE.THRESHOLD and self.gameEngine.positionBlocked(path[index], self.gameEngine.ghosts[self.ghostname]):
            index +=1;
 
        return path[index]['id'], path[len(path)-1]['id']

            
    def cb_commands_robot(self,msg):
        if (msg.data == "init Guess"):
            self.init_guess_srv(0.1)
  
if __name__ == '__main__':
    #if not (len(sys.argv) == 2):
    #    print "usage: controllerWaypoints.py <ghostname>"
    #    sys.exit(-1)

    #ghostname = sys.argv[1]
    rospy.init_node('controller_ghosts')
    controller_ghosts = ControllerGhosts()
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        controller_ghosts.update()
        r.sleep()
    


