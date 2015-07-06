#!/usr/bin/env python

import unittest

import rospy
import roslib
roslib.load_manifest('pandora_fsm')

from state_manager_msgs.msg import RobotModeMsg
from pandora_fsm import Agent


class TestEndState(unittest.TestCase):
    """ Tests for the end state. """

    def setUp(self):
        rospy.init_node('state_end_test')
        self.agent = Agent(strategy='normal')

    def test_global_state_change(self):
        final = RobotModeMsg.MODE_OFF
        self.agent.to_end()

        self.assertEqual(self.agent.state_changer.get_current_state(), final)
        self.assertEqual(self.agent.state, 'off')
