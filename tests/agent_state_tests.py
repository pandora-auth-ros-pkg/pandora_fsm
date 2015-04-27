#!/usr/bin/env python

"""
    Agent state tests. These tests involve only one hop transitions between
    states.
"""

import unittest

import rospy
import roslib
roslib.load_manifest('pandora_fsm')

from rospy import Subscriber, Publisher, sleep
from std_msgs.msg import String

from pandora_fsm import Agent, TimeoutException, TimeLimiter


""" NORMAL strategy """


class TestOffState(unittest.TestCase):
    """ Tests for the off state. """

    def setUp(self):
        self.agent = Agent(strategy='normal')

    def test_sleep_to_init(self):
        self.agent.set_breakpoint('init')
        self.agent.wake_up()

        self.assertEqual(self.agent.state, 'init')


class TestInitState(unittest.TestCase):
    """ Tests for the init state. """

    def setUp(self):
        self.linear_mock = Publisher('mock/linear', String)
        self.effector_mock = Publisher('mock/effector', String)
        self.agent = Agent(strategy='normal')

    def test_init_to_exploration(self):
        self.effector_mock.publish(String('success:1'))
        self.linear_mock.publish(String('success:1'))
        self.agent.set_breakpoint('exploration')
        self.agent.to_init()
        self.assertEqual(self.agent.state, 'exploration')

    def test_initialization_with_linear_failure(self):

        self.effector_mock.publish(String('success:1'))
        self.linear_mock.publish(String('abort:1'))
        self.agent.set_breakpoint('exploration')

        # If the end effector is not responsive the init
        # task will loop forever. Using this decorator
        # we limit the execution time of the task.
        # Wrap your function and test the wrapper.
        @TimeLimiter(timeout=5)
        def init_wrapper():
            self.agent.to_init()
        self.assertRaises(TimeoutException, init_wrapper)

    def test_initialization_with_effector_failure(self):

        self.effector_mock.publish(String('abort:1'))
        self.linear_mock.publish(String('success:1'))
        self.agent.set_breakpoint('exploration')

        # If the end effector is not responsive the init
        # task will loop forever. Using this decorator
        # we limit the execution time of the task.
        # Wrap your function and test the wrapper.
        @TimeLimiter(timeout=5)
        def init_wrapper():
            self.agent.to_init()
        self.assertRaises(TimeoutException, init_wrapper)


class TestExplorationState(unittest.TestCase):
    """ Tests for the exploration state. """

    def setUp(self):
        pass


class TestIdentificationState(unittest.TestCase):
    """ Tests for the identification state. """

    def setUp(self):
        pass


class TestCloseupState(unittest.TestCase):
    """ Tests for the closeup state. """

    def setUp(self):
        pass


class TestVictimDeletionState(unittest.TestCase):
    """ Tests for the victim deletion state. """

    def setUp(self):
        pass


class TestFusionValidationState(unittest.TestCase):
    """ Tests for the fusion validation state. """

    def setUp(self):
        pass


class TestOperatorValidationState(unittest.TestCase):
    """ Tests for the fusion validation state. """

    def setUp(self):
        pass


if __name__ == '__main__':
    rospy.init_node('test_agent_states')
    unittest.main()
