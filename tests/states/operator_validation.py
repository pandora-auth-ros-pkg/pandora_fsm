#!/usr/bin/env python

import unittest

import rospy
import roslib
roslib.load_manifest('pandora_fsm')

from rospy import Publisher
from std_msgs.msg import String, Bool

from pandora_fsm import Agent
import mock_msgs


class TestOperatorValidationState(unittest.TestCase):
    """ Tests for the operator validation state. """

    def setUp(self):
        self.agent = Agent(strategy='normal')
        self.gui = Publisher('mock/validate_gui', String)
        self.gui_result = Publisher('mock/gui_result', Bool)
        self.agent.set_breakpoint('fusion_validation')

    def test_to_fusion_validation_by_success(self):
        self.agent.target = mock_msgs.create_victim_info()
        self.gui.publish('success:1')
        self.agent.to_operator_validation()

        self.assertEqual(self.agent.state, 'fusion_validation')

    def test_to_fusion_validation_by_abort(self):
        self.agent.target = mock_msgs.create_victim_info()
        self.gui.publish('abort:1')
        self.agent.to_operator_validation()

        self.assertEqual(self.agent.state, 'fusion_validation')

    def test_update_victims_valid(self):
        """ Expecting to add the valid victim """

        self.agent.victims_found = 0
        self.gui_result.publish(True)
        self.gui.publish('success:2')
        msg = mock_msgs.create_victim_info(id=5, probability=0.8)
        self.agent.target = msg
        self.agent.to_operator_validation()

        self.assertEqual(self.agent.victims_found, 1)
        self.assertEqual(self.agent.state, 'fusion_validation')

    def test_update_multiple_victims_valid(self):
        """ Expecting to add the valid victim """

        self.agent.victims_found = 0
        self.gui_result.publish(True)
        self.gui.publish('success:2')
        msg = mock_msgs.create_victim_info(id=5, probability=0.8)
        self.agent.target = msg
        self.agent.to_operator_validation()

        self.assertEqual(self.agent.victims_found, 1)

        msg = mock_msgs.create_victim_info(id=6, probability=0.8)
        self.agent.target = msg
        self.agent.to_operator_validation()

        self.assertEqual(self.agent.victims_found, 2)
        self.assertEqual(self.agent.state, 'fusion_validation')

    def test_update_victims_invalid(self):
        """ Expecting to ignore this victim """

        self.agent.victims_found = 0
        self.gui_result.publish(False)
        self.gui.publish('success:2')
        msg = mock_msgs.create_victim_info(id=5, probability=0.2)
        self.agent.target = msg
        self.agent.to_operator_validation()

        self.assertEqual(self.agent.victims_found, 0)
        self.assertEqual(self.agent.state, 'fusion_validation')

    def test_update_victims_aborted(self):
        """ Expecting the number of valid victims to remain the same """

        self.agent.victims_found = 0
        self.gui.publish('abort:2')
        msg = mock_msgs.create_victim_info(id=5, probability=0.2)
        self.agent.target = msg
        self.agent.to_operator_validation()

        self.assertEqual(self.agent.victims_found, 0)
        self.assertEqual(self.agent.state, 'fusion_validation')


if __name__ == '__main__':
    rospy.init_node('operator_validation_state')
    unittest.main()
