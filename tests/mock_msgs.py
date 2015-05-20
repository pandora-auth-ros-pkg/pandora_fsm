#! /usr/bin/env python

"""
    Mock messages factories.
"""

from random import randint, random, uniform

import rospy
import roslib
roslib.load_manifest('pandora_fsm')
from std_msgs.msg import String, Int32

from pandora_fsm import topics

# Messages
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from pandora_data_fusion_msgs.msg import WorldModelMsg, VictimInfoMsg


def create_pose():
    msg = Pose()
    msg.position = create_point()
    msg.orientation = create_quaternion()

    return msg


def create_point():
    msg = Point()
    msg.x = random() * 10
    msg.y = random() * 10
    msg.z = random() * 10

    return msg


def create_quaternion():
    msg = Quaternion()
    msg.x = random() * 10
    msg.y = random() * 10
    msg.z = random() * 10
    msg.w = random() * 10

    return msg


def create_pose_stamped():
    msg = PoseStamped()
    msg.header = rospy.Header()
    msg.pose = create_pose()

    return msg


def create_victim_info(id=None, victim_frame_id=None, sensors=None, valid=None,
                       probability=None):
    msg = VictimInfoMsg()

    msg.id = id if id else randint(0, 100)
    msg.victimFrameId = victim_frame_id if victim_frame_id else 'kinect'
    msg.sensors = sensors if sensors else ['thermal', 'kinect']
    msg.valid = valid if valid else False
    msg.probability = probability if probability else randint(0, 10) * 0.1

    msg.victimPose = create_pose_stamped()

    return msg


def create_world_model(victims=[], visitedVictims=[]):
    msg = WorldModelMsg()
    msg.victims = victims
    msg.visitedVictims = visitedVictims

    return msg
