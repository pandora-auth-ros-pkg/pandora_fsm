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
        self.explorer_mock = Publisher('mock/explorer', String)
        self.agent = Agent(strategy='normal')

    def test_preempt_explorer(self):
        self.explorer_mock.publish('abort:4')
        self.agent.explore()
        self.assertEqual(self.agent.explorer.get_state(), GoalStatus.PENDING)
        self.agent.preempt_exploration()
        self.assertEqual(self.agent.explorer.get_state(), GoalStatus.ABORTED)
        self.assertFalse(self.agent.exploration_done.is_set())

    def test_explorer_abort(self):
        self.explorer_mock.publish('abort:1')
        self.agent.explore()
        sleep(3)
        self.assertEqual(self.agent.explorer.get_state(), GoalStatus.ABORTED)
        self.assertFalse(self.agent.exploration_done.is_set())

    def test_explorer_success(self):
        self.explorer_mock.publish('success:1')
        self.agent.explore()
        sleep(3)
        self.assertEqual(self.agent.explorer.get_state(), GoalStatus.SUCCEEDED)
        self.assertTrue(self.agent.exploration_done.is_set())


if __name__ == '__main__':
    rospy.init_node('unit_explorer')
    unittest.main()
