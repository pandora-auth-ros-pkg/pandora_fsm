#!/usr/bin/env python

import unittest

import rospy
import roslib
roslib.load_manifest('pandora_fsm')

from rospy import Publisher
from std_msgs.msg import String

from pandora_fsm import Agent
from pandora_fsm.mocks import msgs as mock_msgs


class TestFusionValidationState(unittest.TestCase):
    """ Tests for the fusion validation state. """

    def setUp(self):
        rospy.init_node('state_fusion_validation_test')
        self.agent = Agent(strategy='normal')
        self.fusion_validate = Publisher('mock/fusion_validate', String)
        self.target = mock_msgs.create_victim_info(id=1, probability=0.65)
        self.agent.target.set(self.target)
        self.agent.set_breakpoint('exploration')

    def test_true_positive(self):

        self.agent.target.verified.set()
        if not rospy.is_shutdown():
            rospy.sleep(1)
            self.fusion_validate.publish('success:1')
        self.agent.gui_result.victimValid = True
        self.agent.to_fusion_validation()

        self.assertEqual(self.agent.state, 'exploration')
        self.assertFalse(self.agent.gui_result.victimValid)
        self.assertTrue(self.agent.target.is_empty)
        self.assertFalse(self.agent.target.is_verified())

    def test_false_positive(self):

        self.agent.target.verified.set()
        if not rospy.is_shutdown():
            rospy.sleep(1)
            self.fusion_validate.publish('success:1')
        self.agent.gui_result.victimValid = False
        self.agent.to_fusion_validation()

        self.assertEqual(self.agent.state, 'exploration')
        self.assertFalse(self.agent.gui_result.victimValid)
        self.assertTrue(self.agent.target.is_empty)
        self.assertFalse(self.agent.target.is_verified())

    def test_true_negative(self):

        if not rospy.is_shutdown():
            rospy.sleep(1)
            self.fusion_validate.publish('success:1')
        self.agent.gui_result.victimValid = False
        self.agent.to_fusion_validation()

        self.assertEqual(self.agent.state, 'exploration')
        self.assertFalse(self.agent.gui_result.victimValid)
        self.assertTrue(self.agent.target.is_empty)
        self.assertFalse(self.agent.target.is_verified())

    def test_false_negative(self):

        if not rospy.is_shutdown():
            rospy.sleep(1)
            self.fusion_validate.publish('success:1')
        self.agent.gui_result.victimValid = True
        self.agent.to_fusion_validation()

        self.assertEqual(self.agent.state, 'exploration')
        self.assertFalse(self.agent.gui_result.victimValid)
        self.assertTrue(self.agent.target.is_empty)
        self.assertFalse(self.agent.target.is_verified())

    def test_invalid_victim(self):

        if not rospy.is_shutdown():
            rospy.sleep(1)
            self.fusion_validate.publish('abort:1')
        self.agent.gui_result.victimValid = False
        self.agent.to_fusion_validation()

        self.assertEqual(self.agent.state, 'exploration')
        self.assertFalse(self.agent.gui_result.victimValid)
        self.assertTrue(self.agent.target.is_empty)
