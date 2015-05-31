#!/usr/bin/env python

import unittest

import rospy
import roslib
roslib.load_manifest('pandora_fsm')

from rospy import Publisher
from std_msgs.msg import String

from state_manager_msgs.msg import RobotModeMsg
from pandora_fsm import Agent, TimeoutException, TimeLimiter


class TestInitState(unittest.TestCase):
    """ Tests for the init state. """

    def setUp(self):
        rospy.init_node('state_init_test')
        self.effector_mock = Publisher('mock/effector', String)
        self.agent = Agent(strategy='normal')
        self.agent.set_breakpoint('exploration')

    def test_init_to_exploration(self):
        self.effector_mock.publish('success:1')
        self.agent.to_init()
        self.assertEqual(self.agent.state, 'exploration')

    def test_initialization_with_effector_failure(self):

        self.effector_mock.publish('abort:1')

        # If the end effector is not responsive the init
        # task will loop forever. Using this decorator
        # we limit the execution time of the task.
        # Wrap your function and test the wrapper.
        @TimeLimiter(timeout=5)
        def init_wrapper():
            self.agent.to_init()
        self.assertRaises(TimeoutException, init_wrapper)

        self.effector_mock.publish('preempt:1')

        @TimeLimiter(timeout=5)
        def init_wrapper():
            self.agent.to_init()
        self.assertRaises(TimeoutException, init_wrapper)

    def test_global_state_change(self):
        self.effector_mock.publish('success:1')

        initial_state = RobotModeMsg.MODE_OFF
        final_state = RobotModeMsg.MODE_START_AUTONOMOUS
        self.assertEqual(self.agent.state_changer.get_current_state(),
                         initial_state)

        self.agent.wake_up()

        self.assertEqual(self.agent.state_changer.get_current_state(),
                         final_state)
