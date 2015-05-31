#!/usr/bin/env python

import unittest

import rospy
import roslib
roslib.load_manifest('pandora_fsm')

from rospy import Publisher
from std_msgs.msg import String, Bool

from pandora_fsm import Agent
from pandora_fsm.mocks import msgs as mock_msgs


class TestOperatorValidationState(unittest.TestCase):
    """ Tests for the operator validation state. """

    def setUp(self):
        rospy.init_node('operator_validation_state')
        self.agent = Agent(strategy='normal')
        self.gui = Publisher('mock/validate_gui', String)
        self.gui_result = Publisher('mock/gui_result', Bool)
        self.agent.set_breakpoint('fusion_validation')

    def test_to_fusion_validation_by_success(self):
        self.agent.target.set(mock_msgs.create_victim_info())
        if not rospy.is_shutdown():
            rospy.sleep(1)
            self.gui.publish('success:1')
        self.agent.to_operator_validation()

        self.assertEqual(self.agent.state, 'fusion_validation')

    def test_to_fusion_validation_by_abort(self):
        self.agent.target.set(mock_msgs.create_victim_info())
        if not rospy.is_shutdown():
            rospy.sleep(1)
            self.gui.publish('abort:1')
        self.agent.to_operator_validation()

        self.assertEqual(self.agent.state, 'fusion_validation')
        self.assertFalse(self.agent.gui_result.victimValid)

    def test_update_victims_valid(self):
        """ Expecting to add the valid victim """

        if not rospy.is_shutdown():
            rospy.sleep(1)
            self.gui_result.publish(True)
            self.gui.publish('success:2')
        msg = mock_msgs.create_victim_info(id=5, probability=0.8)
        self.agent.target.set(msg)
        self.agent.to_operator_validation()

        self.assertTrue(self.agent.gui_result.victimValid)
        self.assertEqual(self.agent.state, 'fusion_validation')

    def test_update_victims_invalid(self):
        """ Expecting to ignore this victim """

        if not rospy.is_shutdown():
            rospy.sleep(1)
            self.gui_result.publish(False)
            self.gui.publish('success:2')
        msg = mock_msgs.create_victim_info(id=5, probability=0.2)
        self.agent.target.set(msg)
        self.agent.to_operator_validation()

        self.assertEqual(self.agent.state, 'fusion_validation')
        self.assertFalse(self.agent.gui_result.victimValid)
