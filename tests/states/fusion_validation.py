#!/usr/bin/env python

import unittest

import rospy
import roslib
roslib.load_manifest('pandora_fsm')

from rospy import Publisher
from std_msgs.msg import String

from pandora_fsm import Agent
import mock_msgs


class TestFusionValidationState(unittest.TestCase):
    """ Tests for the fusion validation state. """

    def setUp(self):
        self.agent = Agent(strategy='normal')
        self.fusion_validate = Publisher('mock/fusion_validate', String)
        self.target = mock_msgs.create_victim_info(id=1, probability=0.65)
        self.agent.set_breakpoint('exploration')
        self.agent.target = self.target

    def test_valid_victim(self):

        self.fusion_validate.publish('success:1')
        self.agent.gui_result.victimValid = True
        self.agent.to_fusion_validation()

        self.assertEqual(self.agent.state, 'exploration')
        self.assertFalse(self.agent.gui_result.victimValid)
        self.assertIsNone(self.agent.target)

    def test_invalid_victim(self):

        self.fusion_validate.publish('abort:1')
        self.agent.gui_result.victimValid = False
        self.agent.to_fusion_validation()

        self.assertEqual(self.agent.state, 'exploration')
        self.assertFalse(self.agent.gui_result.victimValid)
        self.assertIsNone(self.agent.target)

if __name__ == '__main__':
    rospy.init_node('fusion_validation_state')
    unittest.main()
