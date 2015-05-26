#!/usr/bin/env python

"""
    Unit tests for EndEffector action client.
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


class TestEndEffector(unittest.TestCase):
    """ Tests for the end effector action client. """

    def setUp(self):

        # Register the mock servers.
        self.effector_mock = Publisher('mock/effector', String)
        self.agent = Agent(strategy='normal')

    def test_park_end_effector(self):

        self.effector_mock.publish('abort:1')

        @TimeLimiter(timeout=5)
        def infinite_delay():
            self.agent.park_end_effector()

        self.assertRaises(TimeoutException, infinite_delay)

        self.effector_mock.publish('success:1')
        self.agent.park_end_effector()
        self.assertEqual(self.agent.end_effector_client.get_state(),
                         GoalStatus.SUCCEEDED)

    def test_end_effector(self):

        @TimeLimiter(timeout=5)
        def infinite_delay():
            self.agent.test_end_effector()

        self.assertRaises(TimeoutException, infinite_delay)

        self.effector_mock.publish('success:1')
        self.agent.test_end_effector()
        self.assertEqual(self.agent.end_effector_client.get_state(),
                         GoalStatus.SUCCEEDED)

    def test_scan(self):
        self.effector_mock.publish('abort:1')
        self.agent.scan()
        sleep(3)
        self.assertEqual(self.agent.end_effector_client.get_state(),
                         GoalStatus.ABORTED)

        self.effector_mock.publish('success:1')
        self.agent.scan()
        sleep(3)
        self.assertEqual(self.agent.end_effector_client.get_state(),
                         GoalStatus.SUCCEEDED)

    def test_point_sensors(self):
        self.agent.target_victim = mock_msgs.create_victim_info()
        self.effector_mock.publish('abort:1')
        self.agent.point_sensors()
        sleep(3)
        self.assertEqual(self.agent.end_effector_client.get_state(),
                         GoalStatus.ABORTED)

        self.effector_mock.publish('success:1')
        self.agent.point_sensors()
        sleep(3)
        self.assertEqual(self.agent.end_effector_client.get_state(),
                         GoalStatus.SUCCEEDED)

if __name__ == '__main__':
    rospy.init_node('unit_end_effector')
    unittest.main()
