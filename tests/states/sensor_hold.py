#!/usr/bin/env python

import unittest
from threading import Thread

import rospy
import roslib
roslib.load_manifest('pandora_fsm')

from rospy import sleep

from state_manager_msgs.msg import RobotModeMsg
from pandora_fsm import Agent
import mock_msgs


class TestSensorHoldState(unittest.TestCase):
    """ Tests for the sensor_hold state. """

    def setUp(self):
        self.agent = Agent(strategy='normal')
        self.agent.set_breakpoint('fusion_validation')
        self.agent.set_breakpoint('operator_validation')
        self.world_model = Thread(target=self.send_victim, args=(5,))
        self.agent.target = mock_msgs.create_victim_info(id=1)

    def send_victim(self, delay):
        """ Spawn a thread and send a potential victim instead of using a
            mock Publisher which is not so predictable or easy to configure.
        """
        victim = [mock_msgs.create_victim_info(id=1, probability=0.7)]
        visited = [mock_msgs.create_victim_info() for i in range(0, 3)]

        model = mock_msgs.create_world_model(victim, visited)
        sleep(delay)
        self.agent.receive_world_model(model)

    def test_to_operator_validation(self):
        """ The probability is higher than the VERIFICATION_THRESHOLD. """

        self.agent.VERIFICATION_THRESHOLD = 0.6
        victim = mock_msgs.create_victim_info(id=1, probability=0.5)
        self.agent.target = victim
        self.world_model.start()
        self.agent.to_sensor_hold()

        self.assertEqual(self.agent.state, 'operator_validation')
        self.assertTrue(self.agent.victim_verified.is_set())

    def test_to_fusion_validation(self):
        """ The probability is lower than the VERIFICATION_THRESHOLD. """

        self.agent.VERIFICATION_THRESHOLD = 0.9
        victim = mock_msgs.create_victim_info(id=1, probability=0.5)
        self.agent.target = victim
        self.world_model.start()
        self.agent.to_sensor_hold()

        self.assertEqual(self.agent.state, 'fusion_validation')
        self.assertFalse(self.agent.victim_verified.is_set())

    def test_global_state_change(self):
        """ The global state should be MODE_SENSOR_HOLD """

        final = RobotModeMsg.MODE_SENSOR_HOLD
        self.agent.VERIFICATION_THRESHOLD = 0.6
        victim = mock_msgs.create_victim_info(id=1, probability=0.5)
        self.agent.target = victim
        self.world_model.start()
        self.agent.to_sensor_hold()

        self.assertEqual(self.agent.state_changer.get_current_state(), final)
        self.assertEqual(self.agent.state, 'operator_validation')

if __name__ == '__main__':
    rospy.init_node('sensor_hold_state')
    unittest.main()
