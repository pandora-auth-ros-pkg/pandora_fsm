#!/usr/bin/env python

"""
    Unit tests for GUI action client.
"""

import unittest

import roslib
roslib.load_manifest('pandora_fsm')
import rospy
from rospy import Publisher, sleep
from std_msgs.msg import String, Bool

from actionlib_msgs.msg import GoalStatus

from pandora_fsm.mocks import msgs as mock_msgs
from pandora_fsm import Agent


class TestValidateGUI(unittest.TestCase):
    """ Tests the validate gui client """

    def setUp(self):
        self.agent = Agent(strategy='normal')
        self.agent.machine.add_state('test_validate_gui')
        self.agent.machine.add_transition('operator_responded', 'off',
                                          'test_validate_gui')
        self.validate_gui_mock = Publisher('mock/validate_gui', String)
        self.gui_result = Publisher('mock/gui_result', Bool)
        sleep(2)

    def test_validated_true(self):
        """ The operator has stated this victim isvalid """

        self.agent.set_breakpoint('fusion_validation')
        msg = mock_msgs.create_victim_info(id=5)
        self.agent.target_victim = msg
        self.gui_result.publish(True)
        self.validate_gui_mock.publish('success:2')
        self.agent.wait_for_operator()

        self.assertTrue(self.agent.gui_result.victimValid)

    def test_validated_false(self):
        """ The operator has stated this victim is not valid """

        self.agent.set_breakpoint('fusion_validation')
        msg = mock_msgs.create_victim_info(id=5)
        self.agent.target_victim = msg
        self.gui_result.publish(False)
        self.validate_gui_mock.publish('success:2')
        self.agent.wait_for_operator()

        self.assertFalse(self.agent.gui_result.victimValid)

    def test_validation_aborted(self):

        self.agent.set_breakpoint('fusion_validation')
        self.validate_gui_mock.publish('abort:2')
        msg = mock_msgs.create_victim_info(id=5)
        self.agent.target_victim = msg
        self.agent.wait_for_operator()

        self.assertEqual(self.agent.gui_client.get_state(),
                         GoalStatus.ABORTED)

if __name__ == '__main__':
    rospy.init_node('unit_gui')
    unittest.main()
