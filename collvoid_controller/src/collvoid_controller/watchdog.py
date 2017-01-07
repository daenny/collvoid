#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from stage_ros.msg import Stall
from geometry_msgs.msg import Twist, PoseArray, PoseStamped
from std_msgs.msg import String
from std_msgs.msg import Int32, Bool
from std_srvs.srv import Empty, Trigger
import re


class Watchdog(object):
    STALL = 'stall'
    GROUND_TRUTH_ODOM = 'base_pose_ground_truth'
    CMD_VEL = 'cmd_vel'

    num_rep = 1

    WAIT_FOR_INIT = 3  # Wait time before sending start signal
    MAX_TIME = 60  # Max allowed time before timeout.

    AUTO_MODE = True

    resetting = False

    robots_in_collision = []
    robots_finished = []

    stall_count = 0  # total number of unresolved collisions
    stall_count_resolved = 0  # total number of resolved collisions
    exceeded = False

    wait_for_start = True

    INIT = True

    def __init__(self):
        rospy.loginfo('init watchdog')
        self.NUM_REPETITIONS = rospy.get_param("~num_repetitions")  # How many repititions in total?
        rospy.sleep(8)  # wait 8 sec, so everything is started and all topics are there
        topics = rospy.get_published_topics()
        # stall topics
        stall_subs = []
        for (t, _) in topics:
            if re.match('/robot_[0123456789]+/' + self.STALL, t):
                stall_subs.append(rospy.Subscriber(t, Stall, self.cb_stall))
        rospy.loginfo('subscribed to %d "stall" topics' % len(stall_subs))
        self.num_robots = len(stall_subs)
        # twist topics
        self.done_srvs = []
        for i in range(self.num_robots):
            self.robots_in_collision.append(False)
            self.robots_finished.append(False)
            rospy.Subscriber('robot_%d/' % i + self.GROUND_TRUTH_ODOM, Odometry, self.cb_cmd_vel, i, queue_size=1)
            self.done_srvs.append(rospy.ServiceProxy('robot_%d' % i + '/is_done', Trigger, persistent=True))
        #rospy.Subscriber("/commands_robot", String, self.cb_commands_robots)

        self.obst_subs = []
        i = 0
        for (t, _) in topics:
            if re.match('/obst_[0123456789]+/' + self.GROUND_TRUTH_ODOM, t):
                self.obst_subs.append(rospy.Subscriber(t, Odometry, self.cb_obst, i, queue_size=1))
                i += 1
        self.obst_published = [False] * len(self.obst_subs)

        # publisher for collisions
        self.stall_pub = rospy.Publisher("/stall", Int32, queue_size=1)
        self.stall_resolved_pub = rospy.Publisher("/stall_resolved", Int32, queue_size=1)
        self.exceeded_pub = rospy.Publisher("/exceeded", Bool, queue_size=1)
        self.num_run_pub = rospy.Publisher("/num_run", Int32, queue_size=1)
        self.obst_pub = rospy.Publisher("/obstacles", PoseStamped, queue_size=1)

        self.start_time = rospy.Time.now()
        self.controller = ControllerHeadless()

        if self.AUTO_MODE:
            rospy.sleep(self.WAIT_FOR_INIT)
            self.start()
        self.INIT = False

    def start(self):
        self.num_run_pub.publish(Int32(self.num_rep))
        self.controller.all_init_guess()
        rospy.sleep(2)
        self.controller.all_init_guess()
        rospy.sleep(2)
        for i in range(1):
            self.controller.all_start()

    def cb_obst(self, msg, i):
        if self.obst_published[i]:
            self.obst_subs[i].unregister()
        self.obst_published[i] = True
        pose = PoseStamped()
        pose.pose = msg.pose.pose
        pose.header = msg.header
        self.obst_pub.publish(pose)

    def reset(self):
        if self.resetting:
            return
        self.resetting = True
        self.wait_for_start = True

        self.start_time = rospy.Time.now()
        self.stall_pub.publish(Int32(self.stall_count))
        self.stall_resolved_pub.publish(Int32(self.stall_count_resolved))
        self.exceeded_pub.publish(Bool(self.exceeded))
        self.reset_vars()
        self.controller.reset()

        rospy.sleep(self.WAIT_FOR_INIT)
        self.num_rep += 1
        self.start()

        self.resetting = False
        self.exceeded = False

    def reset_vars(self):
        for i in range(self.num_robots):
            self.robots_in_collision[i] = False
            self.robots_finished[i] = False
        self.stall_count = 0
        self.stall_count_resolved = 0

    def cb_stall(self, msg):
        if self.INIT:
            return
        robotId = int(msg.header.frame_id[7:msg.header.frame_id[1:].index('/') + 1])
        if msg.stall and not self.robots_in_collision[robotId]:
            self.robots_in_collision[robotId] = True
            rospy.loginfo('robot %s is in collision' % msg.header.frame_id)
            self.stall_count += 1
        elif not msg.stall and self.robots_in_collision[robotId]:
            self.stall_count -= 1
            self.stall_count_resolved += 1
            self.robots_in_collision[robotId] = False

    def cb_cmd_vel(self, odom_msg, id):
        if self.resetting:
            return
        msg = odom_msg.twist.twist
        if self.wait_for_start:
            if not(msg.linear.x == 0.0 and msg.angular.z == 0.0):
                self.reset_vars()
                self.start_time = rospy.Time.now()
                self.wait_for_start = False
                rospy.loginfo("received first non 0 speed after wait for start")
            else:
                return
        # only after wait for start is true

        # rospy.loginfo("Got a message")
        if msg.linear.x == 0.0 and msg.angular.z == 0.0:
            try:
                res = self.done_srvs[id]()
                if res.success:
                    self.robots_finished[id] = True
            except rospy.ServiceException as e:
                rospy.logwarn("Could not check if robot %d is done. %s", id, e)
        else:
            self.robots_finished[id] = False
        if min(self.robots_finished) == 1:
            rospy.loginfo("I think all robots reached their goal")
            if self.AUTO_MODE:
                self.reset_or_done()

        if (rospy.Time.now() - self.start_time).to_sec() > self.MAX_TIME and not self.exceeded:
            self.exceeded = True
            rospy.loginfo("max time exceeded, resetting")
            self.reset_or_done()

    def reset_or_done(self):
        if self.resetting:
            return

        if self.num_rep < self.NUM_REPETITIONS:
            self.wait_for_start = True
            self.reset()
        else:
            self.stall_pub.publish(Int32(self.stall_count))
            self.stall_resolved_pub.publish(Int32(self.stall_count_resolved))
            self.exceeded_pub.publish(Bool(self.exceeded))
            rospy.logerr("!" * 20 + "I AM DONE" + "!" * 20)


class ControllerHeadless(object):
    def __init__(self):
        self.pub = rospy.Publisher('/commands_robot', String, queue_size=1)
        self.reset_srv = rospy.ServiceProxy('/reset_positions', Empty)
        self.initialized = True

    def all_start(self):
        string = "all next Goal"
        self.pub.publish(str(string))

    def all_init_guess(self):
        string = "all init Guess"
        self.pub.publish(str(string))

    def reset(self):
        self.pub.publish("all Stop")
        # rospy.sleep(0.2)
        self.reset_srv()
        rospy.sleep(0.2)
        self.pub.publish("all Restart")


if __name__ == '__main__':
    rospy.init_node('Watchdog')
    watchdog = Watchdog()
    rospy.spin()
