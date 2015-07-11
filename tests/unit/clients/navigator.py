#!/usr/bin/env python

"""
    Unit tests for the MoveBase action client.
"""

import unittest

import roslib
roslib.load_manifest('pandora_fsm')
import rospy
from time import sleep
from rospy import Publisher
from std_msgs.msg import String

from actionlib_msgs.msg import GoalStatus

from pandora_fsm.mocks import msgs as mock_msgs
from pandora_fsm import Agent


class TestNavigator(unittest.TestCase):
    """ Tests for the base action client """

    def setUp(self):

        # Register the mock servers.
        rospy.init_node('unit_base_control')
        self.move_base_mock = Publisher('mock/feedback_move_base', String)
        self.agent = Agent(strategy='normal')
        target = mock_msgs.create_victim_info(id=8, probability=0.65)
        self.agent.target_victim = target

    def test_move_base_abort(self):
        if not rospy.is_shutdown():
            sleep(1)
            self.move_base_mock.publish('abort:1')
        random_pose = mock_msgs.create_pose_stamped(x=1, y=2, z=3)
        self.agent.navigator.move_base(random_pose)
        self.agent.navigator.client.wait_for_result()
        self.assertEqual(self.agent.navigator.client.get_state(),
                         GoalStatus.ABORTED)
        self.assertFalse(self.agent.navigator.base_pending.is_set())

    def test_move_base_success(self):
        if not rospy.is_shutdown():
            sleep(1)
            self.move_base_mock.publish('success:1')
        random_pose = mock_msgs.create_pose_stamped(x=1, y=2, z=3)
        self.agent.navigator.move_base(random_pose)
        self.agent.navigator.client.wait_for_result()
        self.assertEqual(self.agent.navigator.client.get_state(),
                         GoalStatus.SUCCEEDED)
        self.assertFalse(self.agent.navigator.base_pending.is_set())

    def test_move_base_preempt(self):
        if not rospy.is_shutdown():
            sleep(1)
            self.move_base_mock.publish('success:1')
        random_pose = mock_msgs.create_pose_stamped(x=1, y=2, z=3)
        self.agent.navigator.move_base(random_pose)
        self.agent.navigator.cancel_all_goals()
        self.assertFalse(self.agent.navigator.base_pending.is_set())
