#!/usr/bin/env python

"""
    Unit tests for the Explorer action client.
"""

import unittest

import roslib
roslib.load_manifest('pandora_fsm')
import rospy
from rospy import Publisher, sleep
from std_msgs.msg import String

from actionlib_msgs.msg import GoalStatus

from pandora_fsm import Agent


class TestExplorer(unittest.TestCase):
    """ Tests for the explorer action client """

    def setUp(self):

        # Register the mock servers.
        rospy.init_node('unit_explorer')
        self.explorer_mock = Publisher('mock/explorer', String)
        self.agent = Agent(strategy='normal')

    def test_explore(self):
        self.explorer_mock.publish('success:4')
        self.agent.explore()
        self.assertTrue(self.agent.explorer.exploration_pending.is_set())
        self.assertEqual(self.agent.explorer.client.get_state(),
                         GoalStatus.PENDING)

    def test_preempt(self):
        self.explorer_mock.publish('success:3')
        self.agent.explore()
        self.agent.preempt_explorer()
        self.assertFalse(self.agent.explorer.exploration_pending.is_set())

    def test_explorer_abort(self):
        self.explorer_mock.publish('abort:1')
        self.agent.explore()
        sleep(3)
        self.assertEqual(self.agent.explorer.client.get_state(),
                         GoalStatus.ABORTED)
        self.assertFalse(self.agent.explorer.exploration_pending.is_set())

    def test_explorer_success(self):
        self.explorer_mock.publish('success:1')
        self.agent.explore()
        sleep(3)
        self.assertEqual(self.agent.explorer.client.get_state(),
                         GoalStatus.SUCCEEDED)
        self.assertFalse(self.agent.explorer.exploration_pending.is_set())
