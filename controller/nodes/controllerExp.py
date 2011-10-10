#!/usr/bin/env python
import roslib; roslib.load_manifest('controller')
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

from controller import controller


    
if __name__ == '__main__':
    #x = np.arange(0, 5, 0.1);
   # y = np.sin(x)
    #plt.plot(x, y)
    #plt.show()
    rospy.init_node('controllerExp')
    app = wx.App()
    frame = controller(None,-1,'Controller')
    
    numRep = 50
    WAIT_FOR_INIT = 10
    rospy.sleep(10)
    MAX_TIME = 40
    for i in range(numRep):
        rospy.sleep(WAIT_FOR_INIT)
        for i in range(5):
            frame.start(None)
        rospy.sleep(MAX_TIME)
        frame.stop(None)
        rospy.sleep(1)
        frame.reset(None)
    

    
