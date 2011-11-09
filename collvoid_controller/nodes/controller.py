#!/usr/bin/env python
import roslib; roslib.load_manifest('collvoid_controller')
import rospy
import commands
import string
try:
    import wx
except ImportError:
    raise ImportError,"The wxPython module is required to run this program"

from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped,PoseStamped
from std_srvs.srv import Empty
from collvoid_local_planner.srv import StateSrv
from collvoid_msgs.msg import PoseTwistWithCovariance

import tf

class controller(wx.Frame):

    def __init__(self,parent,id,title):
        wx.Frame.__init__(self,parent,id,title)
        self.parent = parent
        self.initialize()
        
    def initialize(self):
        sizer = wx.BoxSizer(wx.VERTICAL)
        self.SetSizer(sizer)


#        self.subCommands = rospy.Subscriber("/commands_robot", String, self.cbCommands)
        self.subCommonPositions = rospy.Subscriber("/position_share", PoseTwistWithCovariance, self.cbCommonPositions)
        self.pub = rospy.Publisher('/commands_robot', String)

        self.robotList = []
        self.robotList.append("all")

        self.reset_srv = rospy.ServiceProxy('/stageros/reset', Empty)

        static_sizer = wx.StaticBoxSizer(wx.StaticBox(self, wx.ID_ANY, "Controls"), wx.HORIZONTAL)
        sizer.Add(static_sizer, 0)
         
        start = wx.Button(self,-1,label="Start!")
        static_sizer.Add(start, 0)
        self.Bind(wx.EVT_BUTTON, self.start, start)
        
        stop = wx.Button(self,-1,label="Stop!")
        static_sizer.Add(stop, 0)
        self.Bind(wx.EVT_BUTTON, self.stop, stop)

        reset = wx.Button(self,-1,label="Reset!")
        static_sizer.Add(reset, 0)
        self.Bind(wx.EVT_BUTTON, self.reset, reset)


        grid_sizer = wx.GridBagSizer()
        static_sizer = wx.StaticBoxSizer(wx.StaticBox(self, wx.ID_ANY, "New Goals"), wx.HORIZONTAL)
        static_sizer.Add(grid_sizer, 0)
        sizer.Add(static_sizer, 0)
 
        self.choiceBox = wx.Choice(self,-1,choices=self.robotList)

        grid_sizer.Add(self.choiceBox,(0,0),(1,2),wx.EXPAND)
        self.SetPosition(wx.Point(200,200))
        self.SetSize(wx.Size(600,200))
        
      
        setWPon = wx.Button(self,-1,label="WP Planner On")
        grid_sizer.Add(setWPon, (4,0))
        self.Bind(wx.EVT_BUTTON, self.setWPon, setWPon)

        setWPoff = wx.Button(self,-1,label="WP Planner Off")
        grid_sizer.Add(setWPoff, (4,1))
        self.Bind(wx.EVT_BUTTON, self.setWPoff, setWPoff)

        sendInitGuess = wx.Button(self,-1,label="Send init Guess")
        grid_sizer.Add(sendInitGuess, (5,0))
        self.Bind(wx.EVT_BUTTON, self.sendInitGuess, sendInitGuess)

        sendNextGoal = wx.Button(self,-1,label="Send next Goal")
        grid_sizer.Add(sendNextGoal, (5,1))
        self.Bind(wx.EVT_BUTTON, self.sendNextGoal, sendNextGoal)

        
        setCircleOn = wx.Button(self,-1,label="Circle On")
        grid_sizer.Add(setCircleOn, (6,0))
        self.Bind(wx.EVT_BUTTON, self.setCircleOn, setCircleOn)

        setCircleOff = wx.Button(self,-1,label="Circle Off")
        grid_sizer.Add(setCircleOff, (6,1))
        self.Bind(wx.EVT_BUTTON, self.setCircleOff, setCircleOff)

        grid_sizer.AddGrowableCol(0)
        self.SetSizer(sizer)

        self.Layout()
        self.Fit()
        self.Show(True)

    def setWPon(self,event):
        string = "%s WP On"%self.choiceBox.GetStringSelection()
        self.pub.publish(str(string))

    def setWPoff(self,event):
        string = "%s WP Off"%self.choiceBox.GetStringSelection()
        self.pub.publish(str(string))
        
               
    def sendInitGuess(self,event):
        num = self.choiceBox.GetSelection()
        msg = PoseWithCovarianceStamped()
        msg.pose.pose.position.x = float(self.goalX.GetValue())
        msg.pose.pose.position.y = float(self.goalY.GetValue())
        yaw = float(self.goalAng.GetValue())
        q = tf.transformations.quaternion_from_euler(0,0, yaw, axes='sxyz')
        msg.pose.pose.orientation.x = q[0];
        msg.pose.pose.orientation.y = q[1];
        msg.pose.pose.orientation.z = q[2];
        msg.pose.pose.orientation.w = q[3];
        self.initGuessPubs[num].publish(msg)

    def cbCommonPositions(self,msg):
        if self.robotList.count(msg.robot_id) == 0:
            rospy.loginfo("robot added")
            self.robotList.append(msg.robot_id)
            self.choiceBox.Append(msg.robot_id)


    def setCircleOn(self,event):
        string = "%s Circle On"%self.choiceBox.GetStringSelection()
        self.pub.publish(str(string))
    
    def setCircleOff(self,event):
        string = "%s Circle Off"%self.choiceBox.GetStringSelection()
        self.pub.publish(str(string))

    def sendNextGoal(self,event):
        string = "%s next Goal"%self.choiceBox.GetStringSelection()
        self.pub.publish(str(string))


    def sendInitGuess(self,event):
        string = "%s init Guess"%self.choiceBox.GetStringSelection()
        self.pub.publish(str(string))

    def cbGoal(self,msg):
        self.goalX.SetValue(str(msg.pose.position.x))
        self.goalY.SetValue(str(msg.pose.position.y))
        q = []
        q.append(msg.pose.orientation.x)
        q.append(msg.pose.orientation.y)
        q.append(msg.pose.orientation.z)
        q.append(msg.pose.orientation.w)
        self.goalAng.SetValue(str(tf.transformations.euler_from_quaternion(q)[2]))
    
 
    def stop(self,event):
        string = "%s Stop"%self.choiceBox.GetStringSelection()
        self.pub.publish(str(string))

    def start(self,event):
        string = "%s Start"%self.choiceBox.GetStringSelection()
        self.pub.publish(str(string))

    def reset(self,event):
        self.pub.publish("all Stop")
        rospy.sleep(0.2)
        self.pub.publish("all Restart")
        #rospy.sleep(0.2)
        self.reset_srv()
            
    
if __name__ == '__main__':
    rospy.init_node('controller')
    app = wx.App()
    frame = controller(None,-1,'Controller')

    app.MainLoop()
