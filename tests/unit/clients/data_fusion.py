#!/usr/bin/env python

"""
    Unit tests for the DataFusion action client.
"""

import unittest

import roslib
roslib.load_manifest('pandora_fsm')
import rospy
from rospy import Publisher, sleep
from std_msgs.msg import String

from actionlib_msgs.msg import GoalStatus

from pandora_fsm.mocks import msgs as mock_msgs
from pandora_fsm import Agent, TimeLimiter, TimeoutException


class TestDeleteVictim(unittest.TestCase):

    def setUp(self):
        self.agent = Agent(strategy='normal')
        self.agent.machine.add_state('test_delete_victim')
        self.agent.machine.add_transition('victim_deleted', 'off',
                                          'test_delete_victim')
        self.delete_victim_mock = Publisher('mock/delete_victim', String)
        self.agent.target_victim = mock_msgs.create_victim_info()

    def test_delete_victim_with_abort(self):
        """ If the goal is aborted the agent will keep trying. """

        while not rospy.is_shutdown():
            sleep(2)
            self.delete_victim_mock.publish('abort:1')
            break

        @TimeLimiter(timeout=7)
        def infinite_delay():
            self.agent.delete_victim()

        self.assertRaises(TimeoutException, infinite_delay)
        self.assertEqual(self.agent.state, 'off')

    def test_delete_victim_success(self):

        while not rospy.is_shutdown():
            sleep(2)
            self.delete_victim_mock.publish('success:1')
            break

        self.agent.delete_victim()

        self.assertEqual(self.agent.state, 'test_delete_victim')
        self.assertEqual(self.agent.data_fusion.get_state(),
                         GoalStatus.SUCCEEDED)


if __name__ == '__main__':
    rospy.init_node('unit_data_fusion')
    unittest.main()
