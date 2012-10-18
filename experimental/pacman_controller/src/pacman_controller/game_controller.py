#!/usr/bin/env python
import roslib; roslib.load_manifest('pacman_controller')
import rospy
import string
try:
    import wx
except ImportError:
    raise ImportError,"The wxPython module is required to run this program"

from std_msgs.msg import String
from collvoid_msgs.msg import PoseTwistWithCovariance

class GameController(wx.Frame):

    def __init__(self,parent,id,title):
        wx.Frame.__init__(self,parent,id,title)
        self.parent = parent
        self.initialize()
        
    def initialize(self):
        sizer = wx.BoxSizer(wx.VERTICAL)
        self.SetSizer(sizer)
        self.subCommonPositions = rospy.Subscriber("/position_share", PoseTwistWithCovariance, self.cbCommonPositions)
        self.pub = rospy.Publisher('/commands_robot', String)

        self.robotList = []
        self.robotList.append("all")

#        self.reset_srv = rospy.ServiceProxy('/stageros/reset', Empty)

        static_sizer = wx.StaticBoxSizer(wx.StaticBox(self, wx.ID_ANY, "Controls"), wx.HORIZONTAL)
        sizer.Add(static_sizer, 0)
         
        start = wx.Button(self,-1,label="Start!")
        static_sizer.Add(start, 0)
        self.Bind(wx.EVT_BUTTON, self.makeCommand("Start"), start)
        
        stop = wx.Button(self,-1,label="Stop!")
        static_sizer.Add(stop, 0)
        self.Bind(wx.EVT_BUTTON, self.makeCommand("Stop"), stop)

        reset = wx.Button(self,-1,label="Init!")
        static_sizer.Add(reset, 0)
        self.Bind(wx.EVT_BUTTON, self.makeCommand("Init"), reset)


        grid_sizer = wx.GridBagSizer()
        static_sizer = wx.StaticBoxSizer(wx.StaticBox(self, wx.ID_ANY, "New Goals"), wx.HORIZONTAL)
        static_sizer.Add(grid_sizer, 0)
        sizer.Add(static_sizer, 0)
 
        self.choiceBox = wx.Choice(self,-1,choices=self.robotList)

        grid_sizer.Add(self.choiceBox,(0,0),(1,2),wx.EXPAND)
        self.SetPosition(wx.Point(200,200))
        self.SetSize(wx.Size(600,200))
        
      
        saveMap = wx.Button(self,-1,label="Save Map")
        grid_sizer.Add(saveMap, (4,0))
        self.Bind(wx.EVT_BUTTON, self.makeCommand("Save Map"), saveMap)

#        setWPoff = wx.Button(self,-1,label="WP Planner Off")
#        grid_sizer.Add(setWPoff, (4,1))
#        self.Bind(wx.EVT_BUTTON, self.makeCommand("1"), setWPoff)

        sendInitGuess = wx.Button(self,-1,label="Send init Guess")
        grid_sizer.Add(sendInitGuess, (5,0))
        self.Bind(wx.EVT_BUTTON, self.makeCommand("init Guess"), sendInitGuess)

        sendSetup = wx.Button(self,-1,label="Setup")
        grid_sizer.Add(sendSetup, (5,1))
        self.Bind(wx.EVT_BUTTON, self.makeCommand("Setup"), sendSetup)

        
        setCircleOn = wx.Button(self,-1,label="Pause")
        grid_sizer.Add(setCircleOn, (6,0))
        self.Bind(wx.EVT_BUTTON, self.makeCommand("Pause"), setCircleOn)

        setCircleOff = wx.Button(self,-1,label="Resume")
        grid_sizer.Add(setCircleOff, (6,1))
        self.Bind(wx.EVT_BUTTON, self.makeCommand("Resume"), setCircleOff)

        grid_sizer.AddGrowableCol(0)
        self.SetSizer(sizer)

        self.Layout()
        self.Fit()
        self.Show(True)

        
    def makeCommand(self,string):
        return lambda event, s=string: self.sendCommand(s)
               
    def cbCommonPositions(self,msg):
        if self.robotList.count(msg.robot_id) == 0:
            rospy.loginfo("robot added")
            self.robotList.append(msg.robot_id)
            self.choiceBox.Append(msg.robot_id)

    def sendCommand(self,string):
        self.pub.publish(String(string))

    
if __name__ == '__main__':
    rospy.init_node('game_controller')
    app = wx.App()
    frame = GameController(None,-1,'Game Controller')

    app.MainLoop()
