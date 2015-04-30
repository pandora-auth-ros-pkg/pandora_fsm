#!/usr/bin/env python

"""
    Agent state tests. These tests involve only one hop transitions between
    states.
"""

import unittest

import rospy
import roslib
roslib.load_manifest('pandora_fsm')

from rospy import Publisher, sleep
from std_msgs.msg import String

from pandora_fsm import Agent, TimeoutException, TimeLimiter
import mock_msgs


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
        self.effector_mock = Publisher('mock/effector', String)
        self.move_base_mock = Publisher('mock/move_base', String)
        self.victim_mock = Publisher('mock/victim_probability', String)
        self.agent = Agent(strategy='normal')
        target = mock_msgs.create_victim_info(id=8, probability=0.4)
        self.agent.target_victim = target

    def test_to_closeup_move_base_successful(self):
        self.move_base_mock.publish(String('success:1'))
        self.effector_mock.publish(String('success:1'))
        self.victim_mock.publish('8:0.6')
        self.agent.set_breakpoint('closeup')
        self.agent.to_identification()
        self.assertEqual(self.agent.state, 'closeup')

    def test_to_closeup_move_base_aborted(self):
        self.move_base_mock.publish(String('abort:1'))
        self.effector_mock.publish(String('success:1'))
        self.victim_mock.publish('8:0.9')
        self.agent.set_breakpoint('closeup')
        self.agent.to_identification()
        self.assertEqual(self.agent.state, 'closeup')

    def test_to_victim_deletion(self):
        self.move_base_mock.publish(String('abort:1'))
        self.effector_mock.publish(String('success:1'))
        self.victim_mock.publish('8:0.5')
        self.agent.set_breakpoint('victim_deletion')
        self.agent.to_identification()
        self.assertEqual(self.agent.state, 'victim_deletion')


class TestCloseupState(unittest.TestCase):
    """ Tests for the closeup state. """

    def setUp(self):
        pass


class TestVictimDeletionState(unittest.TestCase):
    """ Tests for the victim deletion state. """

    def setUp(self):
        self.effector_mock = Publisher('mock/effector', String)
        self.delete_victim_mock = Publisher('mock/delete_victim', String)
        self.agent = Agent(strategy='normal')
        target = mock_msgs.create_victim_info(id=8, probability=0.65)
        self.agent.target_victim = target

    def test_delete_victim_success(self):
        self.effector_mock.publish(String('success:1'))
        self.delete_victim_mock.publish('success:2')
        self.agent.set_breakpoint('exploration')
        self.agent.to_victim_deletion()
        self.assertEqual(self.agent.state, 'exploration')

    # in this test we check that the agent correctly stays in the same
    # state if the delete goal fails
    def test_delete_victim_fail(self):
        self.effector_mock.publish(String('success:1'))
        self.delete_victim_mock.publish('abort:1')
        self.delete_victim_mock.publish('success:5')
        self.agent.set_breakpoint('exploration')
        self.agent.to_victim_deletion()
        self.assertEqual(self.agent.state, 'exploration')


class TestFusionValidationState(unittest.TestCase):
    """ Tests for the fusion validation state. """

    def setUp(self):
        self.agent = Agent(strategy='normal')
        self.fusion_validate_mock = Publisher('mock/fusion_validate', String)
        sleep(2)

    def test_to_exploration_instant_success(self):
        self.agent.set_breakpoint('exploration')
        self.fusion_validate_mock.publish(String('success:2'))
        self.agent.target_victim = mock_msgs.create_victim_info()
        self.agent.gui_result.victimValid = True
        self.agent.to_fusion_validation()
        self.assertEqual(self.agent.state, 'exploration')

        ''' we don't care if it's valid or not transition-wise '''
        self.fusion_validate_mock.publish(String('success:2'))
        self.agent.target_victim = mock_msgs.create_victim_info()
        self.agent.gui_result.victimValid = False
        self.agent.to_fusion_validation()
        self.assertEqual(self.agent.state, 'exploration')

    def test_to_exploration_abort_then_success(self):
        self.agent.set_breakpoint('exploration')
        self.fusion_validate_mock.publish(String('abort:1'))
        self.agent.target_victim = mock_msgs.create_victim_info()
        self.agent.gui_result.victimValid = True
        self.fusion_validate_mock.publish(String('success:1'))
        self.agent.to_fusion_validation()
        self.assertEqual(self.agent.state, 'exploration')


class TestOperatorValidationState(unittest.TestCase):
    """ Tests for the operator validation state. """

    def setUp(self):
        self.agent = Agent(strategy='normal')
        self.validate_gui_mock = Publisher('mock/validate_gui', String)
        self.set_gui_result = Publisher('mock/gui_result', String)

    def test_to_fusion_validation_by_success(self):
        self.agent.target_victim = mock_msgs.create_victim_info()
        self.agent.set_breakpoint('fusion_validation')
        self.validate_gui_mock.publish(String('success:1'))
        self.agent.to_operator_validation()
        self.assertEqual(self.agent.state, 'fusion_validation')

    def test_to_fusion_validation_by_abort(self):
        self.agent.target_victim = mock_msgs.create_victim_info()
        self.agent.set_breakpoint('fusion_validation')
        self.validate_gui_mock.publish(String('success:1'))
        self.agent.to_operator_validation()
        self.assertEqual(self.agent.state, 'fusion_validation')

if __name__ == '__main__':
    rospy.init_node('test_agent_states')
    unittest.main()
