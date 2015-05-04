#!/usr/bin/env python

"""
    Agent state tests. These tests involve only one hop transitions between
    states.
"""

import unittest
from threading import Thread

import rospy
import roslib
roslib.load_manifest('pandora_fsm')

from rospy import Publisher, sleep
from std_msgs.msg import String, Bool

from state_manager_msgs.msg import RobotModeMsg
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
        self.agent.set_breakpoint('exploration')

    def test_init_to_exploration(self):
        self.effector_mock.publish('success:1')
        self.linear_mock.publish('success:1')
        self.agent.to_init()
        self.assertEqual(self.agent.state, 'exploration')

    def test_initialization_with_linear_failure(self):

        self.effector_mock.publish('success:1')
        self.linear_mock.publish('abort:1')

        # If the end effector is not responsive the init
        # task will loop forever. Using this decorator
        # we limit the execution time of the task.
        # Wrap your function and test the wrapper.
        @TimeLimiter(timeout=5)
        def init_wrapper():
            self.agent.to_init()
        self.assertRaises(TimeoutException, init_wrapper)

        self.linear_mock.publish('preempt:1')

        @TimeLimiter(timeout=5)
        def init_wrapper():
            self.agent.to_init()
        self.assertRaises(TimeoutException, init_wrapper)

    def test_initialization_with_effector_failure(self):

        self.effector_mock.publish('abort:1')
        self.linear_mock.publish('success:1')

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
        self.linear_mock.publish('success:1')

        initial_state = RobotModeMsg.MODE_OFF
        final_state = RobotModeMsg.MODE_START_AUTONOMOUS
        self.assertEqual(self.agent.state_changer.get_current_state(),
                         initial_state)

        self.agent.wake_up()

        self.assertEqual(self.agent.state_changer.get_current_state(),
                         final_state)


class TestExplorationState(unittest.TestCase):
    """ Tests for the exploration state. """

    def setUp(self):
        self.agent = Agent(strategy='normal')
        self.agent.set_breakpoint('identification')
        self.agent.set_breakpoint('end')
        self.agent.set_breakpoint('init')
        self.effector_mock = Publisher('mock/effector', String)
        self.explorer = Publisher('mock/explorer', String)
        self.world_model = Thread(target=self.send_victim, args=(3,))

    def send_victim(self, delay):
        """ Spawn a thread and send a potential victim instead of using a
            mock Publisher which is not so predictable or easy to configure.
        """
        victim = [mock_msgs.create_victim_info()]
        visited = [mock_msgs.create_victim_info() for i in range(0, 3)]

        model = mock_msgs.create_world_model(victim, visited)
        sleep(delay)
        self.agent.receive_world_model(model)

    def test_to_identification(self):

        # Long goals that will not affect the test.
        self.effector_mock.publish('success:10')
        self.explorer.publish('success:10')

        self.world_model.start()
        self.agent.to_exploration()

        self.assertEqual(self.agent.state, 'identification')

        # The event is cleared after success.
        self.assertFalse(self.agent.potential_victim.is_set())

    def test_to_end(self):
        self.effector_mock.publish('success:10')

        # This goal will move the agent to the end state.
        self.explorer.publish('success:1')

        self.agent.to_exploration()

        self.assertEqual(self.agent.state, 'end')

    def test_long_wait_for_victim(self):

        # Long goals that will not affect the test.
        self.effector_mock.publish('success:20')
        self.explorer.publish('success:20')

        @TimeLimiter(timeout=5)
        def init_wrapper():
            self.agent.to_exploration()
        self.assertRaises(TimeoutException, init_wrapper)

    def test_global_state_change(self):
        """ The global state should be MODE_EXPLORATION_RESCUE """

        self.effector_mock.publish('success:20')
        self.explorer.publish('success:5')
        self.agent.set_breakpoint('init')
        final = RobotModeMsg.MODE_EXPLORATION_RESCUE
        self.agent.to_init()
        self.agent.booted()

        self.assertEqual(self.agent.state_changer.get_current_state(), final)
        self.assertEqual(self.agent.state, 'end')


class TestIdentificationState(unittest.TestCase):
    """ Tests for the identification state. """

    def setUp(self):
        self.effector_mock = Publisher('mock/effector', String)
        self.move_base_mock = Publisher('mock/move_base', String)
        self.victim_mock = Publisher('mock/victim_probability', String)
        self.agent = Agent(strategy='normal')
        target = mock_msgs.create_victim_info(id=8, probability=0.4)
        self.agent.target_victim = target

    def test_global_state_change(self):
        """ The global state should be MODE_IDENTIFICATION. """

        self.move_base_mock.publish('success:3')
        self.effector_mock.publish('success:3')
        if not rospy.is_shutdown():
            self.victim_mock.publish('1:0.6')
        self.agent.set_breakpoint('sensor_hold')
        final = RobotModeMsg.MODE_IDENTIFICATION

        self.agent.to_identification()
        self.assertEqual(self.agent.state, 'sensor_hold')
        self.assertEqual(self.agent.state_changer.get_current_state(), final)

    def test_to_sensor_hold_move_base_successful(self):
        self.move_base_mock.publish('success:1')
        self.effector_mock.publish('success:1')
        self.victim_mock.publish('8:0.6')
        self.agent.set_breakpoint('sensor_hold')
        self.agent.to_identification()
        self.assertEqual(self.agent.state, 'sensor_hold')

    def test_to_sensor_hold_move_base_aborted(self):
        self.move_base_mock.publish('abort:1')
        self.effector_mock.publish('success:1')
        self.victim_mock.publish('8:0.9')
        self.agent.set_breakpoint('sensor_hold')
        self.agent.to_identification()
        self.assertEqual(self.agent.state, 'sensor_hold')

    def test_to_victim_deletion(self):
        self.move_base_mock.publish('abort:1')
        self.effector_mock.publish('success:1')
        self.victim_mock.publish('8:0.5')
        self.agent.set_breakpoint('victim_deletion')
        self.agent.to_identification()
        self.assertEqual(self.agent.state, 'victim_deletion')


class TestSensorHoldState(unittest.TestCase):
    """ Tests for the sensor_hold state. """

    def setUp(self):
        self.agent = Agent(strategy='normal')
        self.agent.set_breakpoint('fusion_validation')
        self.agent.set_breakpoint('operator_validation')
        self.world_model = Thread(target=self.send_victim, args=(5,))
        self.agent.target_victim = mock_msgs.create_victim_info(id=1)

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
        """ The probability of the victim is higher than the
            VERIFICATION_THRESHOLD.
        """
        self.agent.VERIFICATION_TIMEOUT = 7
        self.agent.VERIFICATION_THRESHOLD = 0.5
        self.world_model.start()
        self.agent.to_sensor_hold()

        self.assertEqual(self.agent.state, 'operator_validation')

    def test_to_fusion_validation(self):
        """ The probability of the vicitm is lower than the
            VERIFICATION_THRESHOLD.
        """
        self.agent.VERIFICATION_THRESHOLD = 0.9
        self.world_model.start()
        self.agent.to_sensor_hold()

        self.assertEqual(self.agent.state, 'fusion_validation')

    def test_global_state_change(self):
        """ The global state should be MODE_SENSOR_HOLD """

        final = RobotModeMsg.MODE_SENSOR_HOLD
        self.world_model.start()
        self.agent.to_sensor_hold()

        self.assertEqual(self.agent.state_changer.get_current_state(), final)


class TestVictimDeletionState(unittest.TestCase):
    """ Tests for the victim deletion state. """

    def setUp(self):
        self.effector_mock = Publisher('mock/effector', String)
        self.delete_victim_mock = Publisher('mock/delete_victim', String)
        self.agent = Agent(strategy='normal')
        target = mock_msgs.create_victim_info(id=8, probability=0.65)
        self.agent.target_victim = target

    def test_delete_victim_success(self):
        self.effector_mock.publish('success:1')
        self.delete_victim_mock.publish('success:2')
        self.agent.set_breakpoint('exploration')
        self.agent.to_victim_deletion()
        self.assertEqual(self.agent.state, 'exploration')

    # in this test we check that the agent correctly stays in the same
    # state if the delete goal fails
    def test_delete_victim_fail(self):
        self.effector_mock.publish('success:1')
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
        self.fusion_validate_mock.publish('success:2')
        self.agent.target_victim = mock_msgs.create_victim_info()
        self.agent.gui_result.victimValid = True
        self.agent.to_fusion_validation()
        self.assertEqual(self.agent.state, 'exploration')

        # We don't care if it's valid or not transition-wise
        self.fusion_validate_mock.publish('success:2')
        self.agent.target_victim = mock_msgs.create_victim_info()
        self.agent.gui_result.victimValid = False
        self.agent.to_fusion_validation()
        self.assertEqual(self.agent.state, 'exploration')

    def test_to_exploration_abort_then_success(self):
        self.agent.set_breakpoint('exploration')
        self.fusion_validate_mock.publish('abort:1')
        self.agent.target_victim = mock_msgs.create_victim_info()
        self.agent.gui_result.victimValid = True
        self.fusion_validate_mock.publish('success:1')
        self.agent.to_fusion_validation()
        self.assertEqual(self.agent.state, 'exploration')


class TestOperatorValidationState(unittest.TestCase):
    """ Tests for the operator validation state. """

    def setUp(self):
        self.agent = Agent(strategy='normal')
        self.validate_gui_mock = Publisher('mock/validate_gui', String)
        self.gui_result = Publisher('mock/gui_result', Bool)
        self.agent.set_breakpoint('fusion_validation')

    def test_to_fusion_validation_by_success(self):
        self.agent.target_victim = mock_msgs.create_victim_info()
        self.validate_gui_mock.publish('success:2')
        self.agent.to_operator_validation()

        self.assertEqual(self.agent.state, 'fusion_validation')

    def test_to_fusion_validation_by_abort(self):
        self.agent.target_victim = mock_msgs.create_victim_info()
        self.validate_gui_mock.publish('abort:2')
        self.agent.to_operator_validation()

        self.assertEqual(self.agent.state, 'fusion_validation')

    def test_update_victims_valid(self):
        """ Expecting to add the valid victim """

        self.agent.victims_found = 0
        self.gui_result.publish(True)
        self.validate_gui_mock.publish('success:2')
        msg = mock_msgs.create_victim_info(id=5, probability=0.8)
        self.agent.target_victim = msg
        self.agent.to_operator_validation()

        self.assertEqual(self.agent.victims_found, 1)
        self.assertEqual(self.agent.state, 'fusion_validation')

    def test_update_multiple_victims_valid(self):
        """ Expecting to add the valid victim """

        self.agent.victims_found = 0
        self.gui_result.publish(True)
        self.validate_gui_mock.publish('success:2')
        msg = mock_msgs.create_victim_info(id=5, probability=0.8)
        self.agent.target_victim = msg
        self.agent.to_operator_validation()

        self.assertEqual(self.agent.victims_found, 1)

        msg = mock_msgs.create_victim_info(id=6, probability=0.8)
        self.agent.target_victim = msg
        self.agent.to_operator_validation()

        self.assertEqual(self.agent.victims_found, 2)
        self.assertEqual(self.agent.state, 'fusion_validation')

    def test_update_victims_invalid(self):
        """ Expecting to ignore this victim """

        self.agent.victims_found = 0
        self.gui_result.publish(False)
        self.validate_gui_mock.publish('success:2')
        msg = mock_msgs.create_victim_info(id=5, probability=0.2)
        self.agent.target_victim = msg
        self.agent.to_operator_validation()

        self.assertEqual(self.agent.victims_found, 0)
        self.assertEqual(self.agent.state, 'fusion_validation')

    def test_update_victims_aborted(self):
        """ Expecting the number of valid victims to remain the same """

        self.agent.victims_found = 0
        self.validate_gui_mock.publish('abort:2')
        msg = mock_msgs.create_victim_info(id=5, probability=0.2)
        self.agent.target_victim = msg
        self.agent.to_operator_validation()

        self.assertEqual(self.agent.victims_found, 0)
        self.assertEqual(self.agent.state, 'fusion_validation')


if __name__ == '__main__':
    rospy.init_node('test_agent_states')
    unittest.main()
