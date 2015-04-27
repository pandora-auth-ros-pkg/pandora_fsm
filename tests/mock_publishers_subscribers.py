#! /usr/bin/env python

"""
    Publishers and Subscribers mocks.
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

import mock_msgs


class WorldModel(object):

    def __init__(self, name):
        self._name = name
        self._pub = Publisher(topics.world_model, WorldModelMsg)
        Subscriber('mock/' + self._name, String, self.receive_commands)
        Subscriber('mock/victim_probability', String, self.receive_probability)
        loginfo('+ Starting ' + self._name)

    def receive_commands(self, msg):
        self.frequency = float(msg.data)
        self._rate = rospy.Rate(self.frequency)
        self.send()

    def send(self):
        count = 0
        while not rospy.is_shutdown():
            msg = self.create_msg()
            self._pub.publish(msg)
            count += 1
            sleep(1)
            if count > self.frequency:
                break

    def create_msg(self):
        msg = WorldModelMsg()
        msg.victims = [mock_msgs.create_victim_info() for i in range(randint(1, 3))]
        msg.visitedVictims = [mock_msgs.create_victim_info() for i in range(randint(1, 3))]

        return msg

    def receive_probability(self, msg):
        ident_temp, prob_temp = msg.data.split(':')
        ident = int(ident_temp)
        prob = float(prob_temp)
        self.publish_custom_msg(id=ident, probability=prob)

    def publish_custom_msg(self, id=None, victim_frame_id=None, sensors=None,
                           valid=None, probability=None):
        msg = WorldModelMsg()
        msg.victims = [mock_msgs.create_victim_info(id, victim_frame_id, sensors, valid,
                       probability)]
        rate = rospy.Rate(5)
        timeout_thershold = 7
        self._rate = rate
        self.flag = True

        # We set the threshold and the rate so that the world model is
        # updated in time for the tests to run smoothly
        Timer(Duration(timeout_thershold), self.publish_timeout, True)
        while self.flag:
            self._pub.publish(msg)
            self._rate.sleep()

    def publish_timeout(self, event):
        loginfo("finished publishing")

        self.flag = False

if __name__ == '__main__':
    rospy.init_node('mock_node')
    world = WorldModel('world_model')

    rospy.spin()
