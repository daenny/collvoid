#!/usr/bin/env python
__author__ = 'danielclaes'
import rospy
from collvoid_local_planner.srv import GetCollvoidTwist, GetCollvoidTwistRequest
from geometry_msgs.msg import Twist, PoseStamped
from std_srvs.srv import Empty, EmptyResponse
from socket import gethostname
import tf
import tf.transformations

GLOBAL_FRAME = '/map'


class ActiveCollisionAvoidanceController(object):
    def __init__(self):
        self.hostname = rospy.get_namespace()
        self.active = False
        if self.hostname == "/":
            self.hostname = gethostname()
        self.tf_listener = tf.TransformListener()
        rospy.sleep(0.5)
        self.move_back_to_start = True
        self.home_pose = None
        self.base_link = rospy.get_param("~base_frame_id", "/base_link")
        rospy.loginfo("Name: %s", self.hostname)
        self.get_twist_srv = rospy.ServiceProxy("get_collvoid_twist", GetCollvoidTwist)
        self.twist_publisher = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        rospy.Service("toggle_active_collvoid", Empty, self.toggle_active_collvoid)

    def toggle_active_collvoid(self, req):
        self.home_pose = self.get_own_pose()
        self.active = not self.active
        return EmptyResponse()

    def get_own_pose(self):
        own_pose = PoseStamped()
        own_pose.header.frame_id = self.base_link
        own_pose.header.stamp = rospy.Time.now()
        own_pose.pose.orientation.w = 1
        try:
            self.tf_listener.waitForTransform(GLOBAL_FRAME, self.base_link, own_pose.header.stamp, rospy.Duration(0.2))
            result = self.tf_listener.transformPose(GLOBAL_FRAME, own_pose)
        except tf.Exception as e:
            rospy.logwarn("%s: could not transform pose, %s", self.hostname, e)
            return None
        return result

    def get_twist(self):
        req = GetCollvoidTwistRequest()
        if self.move_back_to_start:
            req.goal = self.home_pose
        else:
            req.goal = self.get_own_pose()
        if req.goal is None:
            return Twist()
        try:
            res = self.get_twist_srv(req)
        except rospy.ServiceException as e:
            rospy.logwarn("%s could not get twist %s", self.hostname, e)
            return Twist()
        return res.twist

    def spin(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.active:
                self.twist_publisher.publish(self.get_twist())
            r.sleep()


if __name__ == '__main__':
    rospy.init_node('active_collision_avoidance')
    active_collvoid = ActiveCollisionAvoidanceController()
    active_collvoid.spin()
