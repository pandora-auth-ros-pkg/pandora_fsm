#! /usr/bin/env python

"""
    Mock Publishers.
"""

from random import randint, random

import rospy
from rospy import loginfo, sleep, Publisher, Subscriber, Timer, Duration
import roslib
roslib.load_manifest('pandora_fsm')
from std_msgs.msg import String, Int32

from pandora_fsm import topics

# Messages
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from pandora_data_fusion_msgs.msg import WorldModelMsg, VictimInfoMsg

import msgs


class WorldModel(object):
    """ Mock publisher for the world_model. """

    def __init__(self):
        self._pub = Publisher(topics.world_model, WorldModelMsg)

    def send_random(self, duration=3, new_victims=2, old_victims=5):
        msg = WorldModelMsg()
        msg.victims = [msgs.create_victim_info() for i in range(new_victims)]
        msg.visitedVictims = [msgs.create_victim_info() for i in range(old_victims)]

        for i in range(duration):
            if not rospy.is_shutdown():
                self._pub.publish(msg)
                sleep(1)

    def send_custom(self, victims, visited, duration=10):
        msg = WorldModel()
        msg.victims = victims
        msg.visitedVictims = visited

        for i in range(duration):
            if not rospy.is_shutdown():
                self._pub.publish(msg)
                sleep(1)
