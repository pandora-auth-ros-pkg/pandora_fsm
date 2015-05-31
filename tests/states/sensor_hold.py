#!/usr/bin/env python

import unittest
from threading import Thread

import rospy
import roslib
roslib.load_manifest('pandora_fsm')

from rospy import sleep

from state_manager_msgs.msg import RobotModeMsg
from pandora_fsm import Agent
from pandora_fsm.mocks import msgs as mock_msgs
import pandora_fsm.config as conf


class TestSensorHoldState(unittest.TestCase):
    """ Tests for the sensor_hold state. """

    def setUp(self):
        rospy.init_node('state_sensor_hold_test')
        self.agent = Agent(strategy='normal')
        self.agent.set_breakpoint('fusion_validation')
        self.agent.set_breakpoint('operator_validation')
        self.agent.target.set(mock_msgs.create_victim_info(id=1))

    def send_victim(self, delay, probability):
        """ Spawn a thread and send a potential victim instead of using a
            mock Publisher which is not so predictable or easy to configure.
        """
        victim = [mock_msgs.create_victim_info(id=1, probability=probability)]
        visited = [mock_msgs.create_victim_info() for i in range(0, 3)]

        model = mock_msgs.create_world_model(victim, visited)
        sleep(delay)
        self.agent.receive_world_model(model)

    def test_to_operator_validation(self):
        """ The probability is higher than the VERIFICATION_THRESHOLD. """

        conf.VERIFICATION_THRESHOLD = 0.6
        Thread(target=self.send_victim, args=(1, 0.9, )).start()
        self.agent.to_sensor_hold()

        self.assertEqual(self.agent.state, 'operator_validation')
        self.assertTrue(self.agent.target.is_verified())

    def test_to_fusion_validation(self):
        """ The probability is lower than the VERIFICATION_THRESHOLD. """

        self.agent.VERIFICATION_THRESHOLD = 0.9
        Thread(target=self.send_victim, args=(1, 0.7, )).start()
        self.agent.to_sensor_hold()

        self.assertEqual(self.agent.state, 'fusion_validation')
        self.assertFalse(self.agent.target.is_verified())

    def test_global_state_change(self):
        """ The global state should be MODE_SENSOR_HOLD """

        final = RobotModeMsg.MODE_SENSOR_HOLD
        self.agent.VERIFICATION_THRESHOLD = 0.6
        self.agent.to_sensor_hold()

        self.assertEqual(self.agent.state_changer.get_current_state(), final)
