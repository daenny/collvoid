#!/usr/bin/env python
from geometry_msgs.msg import PoseStamped, PolygonStamped, Point32

__author__ = 'danielclaes'
import rospy
import tf
from spencer_tracking_msgs.msg import TrackedPersons, TrackedPerson
from collvoid_msgs.msg import AggregatedPoseTwist, PoseTwistWithCovariance
from math import sin, cos, pi

global_frame = "/map"
radius = 0.4
division = 4

class PeopleTracker(object):
    def __init__(self):
        self.detected = []
        self.people_pub = rospy.Publisher('people', AggregatedPoseTwist, queue_size=1)
        self.tf_listener = tf.TransformListener()
        self.footprint = PolygonStamped()
        num_points = 32
        for i in xrange(num_points):
            p = Point32()
            p.x = radius * cos((2. * pi * i) / num_points)
            p.y = radius * sin((2. * pi * i) / num_points)
            self.footprint.polygon.points.append(p)
        rospy.sleep(0.5)
        rospy.Subscriber('tracked_persons', TrackedPersons, self.detected_persons)

    def detected_persons(self, msg):
        """
        :param msg:
        :type msg: TrackedPersons
        :return:
        """
        publish_msg = AggregatedPoseTwist()
        publish_msg.header.stamp = msg.header.stamp
        publish_msg.header.frame_id = global_frame
        for track in msg.tracks:
            person = PoseTwistWithCovariance()
            person.header.stamp = msg.header.stamp
            person.header.frame_id = global_frame
            person.controlled = False
            person.twist = track.twist
            person.twist.twist.linear.x /= division
            person.twist.twist.linear.y /= division
            person.twist.twist.angular.z /= division
            person.holo_robot = True
            person.robot_id = "person_" + str(track.track_id)
            person.footprint = self.footprint
            person.radius = radius
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose = track.pose.pose
            pose = self.transform_pose_to_global(pose)
            person.pose.pose = pose.pose
            person.pose.covariance = track.pose.covariance
            if pose is None:
                continue
            publish_msg.posetwists.append(person)
        self.people_pub.publish(publish_msg)

    def transform_pose_to_global(self, pose):
        try:
            self.tf_listener.waitForTransform(pose.header.frame_id, global_frame, pose.header.stamp, rospy.Duration(0.1))
            result = self.tf_listener.transformPose(global_frame, pose)
        except tf.Exception as e:
            rospy.logwarn("Peopletracker: could not transform pose, %s", e)
            return None
        return result


if __name__ == '__main__':
    rospy.init_node('people_tracker')
    tracker = PeopleTracker()
    rospy.spin()

