#!/usr/bin/env python
__author__ = 'daniel'

import rospy
import importlib
from socket import gethostname

import lcm
import StringIO

# from msg import communication_msg

lc = None
lcm_channel = None
TYPE = None
name = None
lcm_sub = None
pub = None
id = None
module_type = None


def init_globals():
    global lc, name, pub, lcm_sub, lcm_channel, TYPE, module_type, id
    lc = lcm.LCM("udpm://224.1.1.1:5007?ttl=2")
    name = gethostname()
    # lcm_sub = lc.subscribe(name, udp_callback)
    # if 'tb' in name:
    topic = rospy.get_param('~topic')
    lcm_channel = rospy.get_param('~lcm_channel', topic)

    msg_package = rospy.get_param('~msg_package')
    msg_name = rospy.get_param('~msg_name')
    module_type = rospy.get_param('~module_type')

    module = importlib.import_module(msg_package)
    TYPE = getattr(module, msg_name)
    id = rospy.get_param('~id', 'robot_id')

    if module_type in ['receiver', 'transceiver']:
        lcm_sub = lc.subscribe(lcm_channel, udp_callback)
        pub = rospy.Publisher(topic, TYPE, queue_size=1)

    if module_type in ['sender', 'transceiver']:
        rospy.Subscriber(topic, TYPE, handle_msg)


def udp_callback(channel, data):
    msg = TYPE()
    msg.deserialize(data)
    if module_type == 'transceiver' and getattr(msg, id) == name:
        return
    pub.publish(msg)


def handle_msg(msg):
    if module_type == 'transceiver' and getattr(msg, id) != name:
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
        lc.publish(lcm_channel, buff.getvalue())


if __name__ == '__main__':
    main()
