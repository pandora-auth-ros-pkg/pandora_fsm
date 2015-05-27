#!/usr/bin/env python

"""
    Unit tests for the MoveBase action client.
"""

import unittest

import roslib
roslib.load_manifest('pandora_fsm')
import rospy
from rospy import Publisher
from std_msgs.msg import String

from actionlib_msgs.msg import GoalStatus

from pandora_fsm.mocks import msgs as mock_msgs
from pandora_fsm import Agent


class TestMoveBase(unittest.TestCase):
    """ Tests for the base action client """

    def setUp(self):

        # Register the mock servers.
        self.move_base_mock = Publisher('mock/move_base', String)
        self.agent = Agent(strategy='normal')
        target = mock_msgs.create_victim_info(id=8, probability=0.65)
        self.agent.target_victim = target

    def test_move_base_abort(self):
        self.move_base_mock.publish('abort:1')
        self.agent.move_base()
        self.agent.navigator.wait_for_result()
        self.assertEqual(self.agent.navigator.get_state(),
                         GoalStatus.ABORTED)

    def test_move_base_success(self):
        self.move_base_mock.publish('success:1')
        self.agent.move_base()
        self.agent.navigator.wait_for_result()
        self.assertEqual(self.agent.navigator.get_state(),
                         GoalStatus.SUCCEEDED)

    def test_move_base_preempt(self):
        self.agent.move_base()
        self.agent.preempt_move_base()
        self.assertEqual(self.agent.navigator.get_state(),
                         GoalStatus.ABORTED)


if __name__ == '__main__':
    rospy.init_node('unit_base_control')
    unittest.main()
