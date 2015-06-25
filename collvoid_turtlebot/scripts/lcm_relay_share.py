#!/usr/bin/env python
__author__ = 'daniel'

import rospy
import importlib
from socket import gethostname

import lcm
import StringIO

# from msg import communication_msg

lc = None
topic = '/position_share'
lcm_topic ='tb'
msg_name = 'PoseTwistWithCovariance'
package = 'collvoid_msgs.msg'

module = importlib.import_module(package)
TYPE = getattr(module, msg_name)


name = None
lcm_sub = None
pub = None


def init_globals():
    global lc, name, pub, lcm_sub
    lc = lcm.LCM("udpm://224.1.1.1:5007?ttl=2")
    name = gethostname()
    print name
    #lcm_sub = lc.subscribe(name, udp_callback)
    #if 'tb' in name:
    lcm_sub = lc.subscribe(lcm_topic, udp_callback)
    pub = rospy.Publisher(topic, TYPE, queue_size=1)
    rospy.Subscriber(topic, TYPE, handle_msg)


def udp_callback(channel, data):
    msg = TYPE()
    msg.deserialize(data)
    if msg.robot_id != name:
        pub.publish(msg)


def handle_msg(msg):
    if msg.robot_id != name:
        return
    send(msg)


def main():
    rospy.init_node('share_lcm_relay')
    init_globals()

    while not rospy.is_shutdown():
        lc.handle()
    lc.unsubscribe(lcm_sub)


def send(msg, repeats=1):
    buff = StringIO.StringIO()
    msg.serialize(buff)
    for i in xrange(repeats):
        lc.publish(lcm_topic, buff.getvalue())


if __name__ == '__main__':
    main()
