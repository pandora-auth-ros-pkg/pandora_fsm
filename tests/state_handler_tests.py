#!/usr/bin/env python

"""
 Tests for the RobotStateHandler and the transitios between the global states.
"""

import unittest

import rospy
import roslib
roslib.load_manifest('pandora_fsm')

from pandora_fsm import RobotStateHandler
from pandora_fsm import Agent


class RobotStateTest(unittest.TestCase):
    """ Tests for the RobotStateHanlder. """

    def setUp(self):

        # Create an agent
        self.agent = Agent(config='testing.json', strategy='empty')

        # Create a hanlder
        self.handler = RobotStateHandler(self.agent)

    def test_agent_starts(self):

        self.handler.start_agent()
        self.assertEqual(self.agent.state, 'init')

    def test_agent_stops(self):

        self.handler.stop_agent()
        self.assertEqual(self.agent.state, 'off')

if __name__ == '__main__':
    rospy.init_node('state_handler_tests')
    unittest.main()
