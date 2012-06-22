#!/usr/bin/env python

import roslib
roslib.load_manifest('collvoid_controller')
import rospy
roslib.load_manifest('stage')
import string
from stage.msg import Stall
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Int32,Bool
from geometry_msgs.msg import PoseWithCovarianceStamped,PoseStamped
from std_srvs.srv import Empty
from collvoid_msgs.msg import PoseTwistWithCovariance

#from collvoid_controller import controller


import re


class Watchdog():
    
    STALL = 'stall'
    ODOM = 'base_pose_ground_truth'
    CMD_VEL = 'cmd_vel'
    X_MAX = 25
    X_MIN = - X_MAX
    Y_MAX = 25
    Y_MIN = - Y_MAX

    NUM_REPITITIONS = 50 #How many repititions in total?
    num_rep = 1

    WAIT_FOR_INIT = 5 #Wait time before sending start signal
    MAX_TIME = 60 #Max allowed time before timeout.

    AUTO_MODE = True

    resetting = False
    
    robots_in_collision = []
    robots_moving = []

    stall_count = 0 #total number of unresolved collisions 
    stall_count_resolved = 0 #total number of resolved collisions 
    exceeded = False
    
    wait_for_start = True

    INIT = True

    def __init__(self):
        rospy.loginfo('init watchdog')
        rospy.sleep(10) #wait 10 sec, so everything is started and all topics are there
        topics = rospy.get_published_topics()
        # stall topics
        stall_subs = []
        for (t,_) in topics:
            if re.match('/robot_[0123456789]+/' + self.STALL, t):
                stall_subs.append(rospy.Subscriber(t, Stall, self.cb_stall))
        rospy.loginfo('subscribed to %d "stall" topics'%len(stall_subs))
        self.num_robots = len(stall_subs)
        # twist topics
        cmd_vel_subs = []
        for i in range(self.num_robots):
            self.robots_in_collision.append(False)
            self.robots_moving.append(True)
            cmd_vel_subs.append(rospy.Subscriber('robot_%d/'%i + self.CMD_VEL, Twist, self.cb_cmd_vel,i))
        # odom topics
        odom_subs = []
        for (t,_) in topics:
            if re.match('/robot_[0123456789]+/' + self.ODOM, t):
                odom_subs.append(rospy.Subscriber(t, Odometry, self.cb_odom))
    
        commands_robots_subs = rospy.Subscriber("/commands_robot",String, self.cb_commands_robots)
        
        #publisher for collisions
        self.stall_pub = rospy.Publisher("/stall", Int32)
        self.stall_resolved_pub = rospy.Publisher("/stall_resolved", Int32)
        self.exceeded_pub = rospy.Publisher("/exceeded", Bool)
        self.num_run_pub = rospy.Publisher("/num_run", Int32)
        rospy.loginfo('subscribed to %d "odom" topics'%len(odom_subs))


        #app.wx.App()
        self.controller = controllerHeadless()
       


        if self.AUTO_MODE:
            rospy.sleep(5)
            self.start()
        self.INIT = False

    def start(self):
        self.num_run_pub.publish(Int32(self.num_rep))
        self.controller.all_init_guess(None)
        rospy.sleep(2)
        self.controller.all_init_guess(None)
        rospy.sleep(2)
        for i in range(1):
            self.controller.all_start(None)

    def reset(self):
        if (self.resetting):
            return
        self.resetting = True
        rospy.sleep(1)
        self.stall_pub.publish(Int32(self.stall_count))
        self.stall_resolved_pub.publish(Int32(self.stall_count_resolved))
        self.exceeded_pub.publish(Bool(self.exceeded))
        self.controller.reset(None)
        self.reset_vars()
        rospy.sleep(self.WAIT_FOR_INIT)
        self.num_rep += 1
        self.start()
        self.resetting = False
        self.exceeded = False
   

    def reset_vars(self):
        for i in range(self.num_robots):
            self.robots_in_collision[i] = False
            self.robots_moving[i] = True
        self.stall_count = 0
        self.stall_count_resolved = 0


    def cb_commands_robots(self, msg):
        if (msg.data == "all Start"):
            self.reset_vars()
            self.start_time = rospy.Time.now()
            rospy.sleep(1)
            self.wait_for_start = False


    def cb_stall(self, msg):
        if self.INIT:
            return
        robotId = int(msg.header.frame_id[7:msg.header.frame_id[1:].index('/')+1])
        if msg.stall and not self.robots_in_collision[robotId]:
            self.robots_in_collision[robotId] = True
            rospy.loginfo('robot %s is in collision'%msg.header.frame_id)
            self.stall_count+=1
        elif not msg.stall and self.robots_in_collision[robotId]:
            self.stall_count-=1
            self.stall_count_resolved+=1
            self.robots_in_collision[robotId] = False
        
    def cb_cmd_vel(self, msg, id):
        if self.wait_for_start or self.resetting:
            return
        #rospy.loginfo("Got a message")
        if msg.linear.x == 0.0 and msg.angular.z == 0.0:
            self.robots_moving[id] = False
            #rospy.loginfo("Robot %d stopped"%id)
        else:
            self.robots_moving[id] = True
        if max(self.robots_moving) == 0:
            rospy.loginfo("I think all robots reached their goal")
            if self.AUTO_MODE:
                if self.num_rep < self.NUM_REPITITIONS:
                    self.wait_for_start = True
                    self.reset()
                elif not self.resetting:
                    self.stall_pub.publish(Int32(self.stall_count))
                    self.stall_resolved_pub.publish(Int32(self.stall_count_resolved))
                    self.exceeded_pub.publish(Bool(self.exceeded))
                    rospy.logerr("!"*20 + "I AM DONE" + "!"*20)

        if (rospy.Time.now() - self.start_time).to_sec() > self.MAX_TIME and not self.exceeded:
            self.exceeded = True
            rospy.loginfo("max time exceeded, resetting")
            if self.num_rep < self.NUM_REPITITIONS:
                self.wait_for_start = True
                self.start_time = rospy.Time.now()

                self.reset()
            elif not self.resetting:
                self.stall_pub.publish(Int32(self.stall_count))
                self.stall_resolved_pub.publish(Int32(self.stall_count_resolved))
                self.exceeded_pub.publish(Bool(self.exceeded))
                rospy.logerr("!"*20 + "I AM DONE" + "!"*20)


    def cb_odom(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y


class controllerHeadless():

    def __init__(self):
        self.pub = rospy.Publisher('/commands_robot', String)
        self.reset_srv = rospy.ServiceProxy('/stageros/reset', Empty)
    #    self.subCommonPositions = rospy.Subscriber("/position_share", PoseTwistWithCovariance, self.cbCommonPositions)
        self.initialized = True

    def cbCommonPositions(self,msg):
        if not self.initialized:
            return
        if self.robotList.count(msg.robot_id) == 0:
            rospy.loginfo("robot added")
            self.robotList.append(msg.robot_id)
    
    def all_start(self,event):
        string = "all Start"
        self.pub.publish(str(string))

    def all_init_guess(self,event):
        string = "all init Guess"
        self.pub.publish(str(string))

        
    def reset(self,event):
        self.pub.publish("all Stop")
        rospy.sleep(0.2)
        self.pub.publish("all Restart")
        #rospy.sleep(0.2)
        self.reset_srv()





if __name__ == '__main__':
    rospy.init_node('watchdog')
    #app = wx.App()
    watchdog = Watchdog()
    #app.MainLoop()
    rospy.spin()
