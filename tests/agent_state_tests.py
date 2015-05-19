#!/usr/bin/env python

"""
    Agent state tests. These tests involve only one hop transitions between
    states.
"""

import unittest
from threading import Thread

from mock import patch

import rospy
import roslib
roslib.load_manifest('pandora_fsm')

from rospy import Publisher, sleep
from std_msgs.msg import String, Bool

from state_manager_msgs.msg import RobotModeMsg
from pandora_fsm import Agent, TimeoutException, TimeLimiter
import mock_msgs

# The sleep is necessary because the time needed for the agent to communicate
# with the action servers is not always the same. We pick a big enough
# number to be safe.

""" NORMAL strategy """


class TestOffState(unittest.TestCase):
    """ Tests for the off state. """

    def setUp(self):
        self.agent = Agent(strategy='normal')

    def test_sleep_to_init(self):
        self.agent.set_breakpoint('init')
        self.agent.wake_up()

        self.assertEqual(self.agent.state, 'init')


class TestEndState(unittest.TestCase):
    """ Tests for the end state. """

    def setUp(self):
        self.agent = Agent(strategy='normal')

    def test_global_state_change(self):
        final = RobotModeMsg.MODE_TERMINATING
        self.agent.to_end()

        self.assertEqual(self.agent.state_changer.get_current_state(), final)
        self.assertEqual(self.agent.state, 'off')


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
        visited = [mock_msgs.create_victim_info() for i in range(0, 3)]
        victim = [mock_msgs.create_victim_info()]

        model = mock_msgs.create_world_model(victim, visited)
        sleep(delay)
        self.agent.receive_world_model(model)

    def test_to_identification(self):

        # Long goals that will not affect the test.
        if not rospy.is_shutdown():
            sleep(1)
            self.effector_mock.publish('success:20')
            self.explorer.publish('success:20')
        self.world_model.start()
        self.agent.to_exploration()
        sleep(10)
        self.assertEqual(self.agent.state, 'identification')
        self.assertFalse(self.agent.state_can_change.is_set())

    def test_race_condition(self):
        if not rospy.is_shutdown():
            sleep(1)
            self.explorer.publish('success:1')
            self.world_model.start()
        self.agent.to_exploration()
        sleep(10)
        self.assertFalse(self.agent.state_can_change.is_set())
        self.assertEqual(self.agent.state, 'end')

    def test_to_end(self):
        self.effector_mock.publish('success:10')

        # This goal will move the agent to the end state.
        if not rospy.is_shutdown():
            sleep(1)
            self.explorer.publish('success:1')

        self.agent.to_exploration()
        sleep(15)
        self.assertEqual(self.agent.state, 'end')
        self.assertFalse(self.agent.state_can_change.is_set())

    def test_long_wait_for_victim(self):

        # Long goals that will not affect the test.
        if not rospy.is_shutdown():
            sleep(1)
            self.effector_mock.publish('success:20')
            self.explorer.publish('success:40')

        self.agent.to_exploration()
        sleep(20)
        self.assertEqual(self.agent.state, 'exploration')
        self.assertTrue(self.agent.state_can_change.is_set())

    def test_retry_on_explorer_abort(self):
        """ The agent will keep sending goals if the explorer fails. """

        if not rospy.is_shutdown():
            sleep(1)
            self.explorer.publish('abort:1')
            self.explorer.publish('abort:1')

        with patch.object(self.agent.explorer.dispatcher, 'emit') as mock:
            self.agent.to_exploration()
            sleep(7)
        mock.assert_called_with('exploration.retry')
        self.assertEqual(self.agent.state, 'exploration')
        self.assertTrue(self.agent.state_can_change.is_set())

    def test_retry_on_explorer_reject(self):
        """ The agent will keep sending goals if the explorer fails. """

        if not rospy.is_shutdown():
            sleep(1)
            self.explorer.publish('reject:1')
            self.explorer.publish('reject:1')
        with patch.object(self.agent.explorer.dispatcher, 'emit') as mock:
            self.agent.to_exploration()
            sleep(7)
        mock.assert_called_with('exploration.retry')
        self.assertEqual(self.agent.state, 'exploration')
        self.assertTrue(self.agent.state_can_change.is_set())

    def test_global_state_change(self):
        """ The global state should be MODE_EXPLORATION_RESCUE """

        if not rospy.is_shutdown():
            sleep(1)
            self.effector_mock.publish('success:20')
            self.explorer.publish('success:1')
        final = RobotModeMsg.MODE_EXPLORATION_RESCUE
        self.agent.to_exploration()
        sleep(10)

        self.assertEqual(self.agent.state_changer.get_current_state(), final)
        self.assertEqual(self.agent.state, 'end')
        self.assertFalse(self.agent.state_can_change.is_set())


class TestIdentificationState(unittest.TestCase):
    """ Tests for the identification state. """

    def setUp(self):
        self.effector_mock = Publisher('mock/effector', String)
        self.move_base_mock = Publisher('mock/feedback_move_base', String)
        self.victim_mock = Publisher('mock/victim_probability', String)
        self.agent = Agent(strategy='normal')
        self.agent.set_breakpoint('victim_deletion')
        self.agent.set_breakpoint('sensor_hold')
        target = mock_msgs.create_victim_info(id=8, probability=0.4)
        self.agent.target_victim = target

    def test_global_state_change(self):
        """ The global state should be MODE_IDENTIFICATION. """

        self.move_base_mock.publish('success:2')
        if not rospy.is_shutdown():
            self.victim_mock.publish('5:0.6')
        final = RobotModeMsg.MODE_IDENTIFICATION
        self.agent.to_identification()
        sleep(5)

        self.assertEqual(self.agent.state, 'sensor_hold')
        self.assertEqual(self.agent.state_changer.get_current_state(), final)

    def test_found_victim(self):
        """ The base has moved to the victim's pose. """

        self.move_base_mock.publish('success:1')
        if not rospy.is_shutdown():
            self.victim_mock.publish('3:0.6')
        self.agent.to_identification()
        sleep(5)
        self.assertEqual(self.agent.state, 'sensor_hold')

    def test_abort_with_high_probability(self):
        """ The move_base has failed but the victim has high probability. """

        self.agent.IDENTIFICATION_THRESHOLD = 0.6
        self.move_base_mock.publish('abort:2')
        if not rospy.is_shutdown():
            self.victim_mock.publish('5:0.9')
        self.agent.to_identification()
        sleep(15)

        self.assertEqual(self.agent.state, 'sensor_hold')
        self.assertTrue(self.agent.probable_victim.is_set())

    def test_abort_with_convergence(self):
        """ The agent is close enough to the victim to be identified. """

        self.move_base_mock.publish('abort_feedback:2')
        self.agent.IDENTIFICATION_THRESHOLD = 0.7
        if not rospy.is_shutdown():
            self.victim_mock.publish('5:0.6')
        self.agent.to_identification()
        sleep(20)

        self.assertEqual(self.agent.state, 'sensor_hold')
        self.assertTrue(self.agent.base_converged.is_set())

    def test_abort_victim(self):
        self.move_base_mock.publish('abort:1')
        if not rospy.is_shutdown():
            self.victim_mock.publish('8:0.5')
        self.agent.to_identification()
        sleep(10)

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


class TestVictimDeletionState(unittest.TestCase):
    """ Tests for the victim deletion state. """

    def setUp(self):
        self.delete_victim_mock = Publisher('mock/delete_victim', String)
        self.agent = Agent(strategy='normal')
        self.agent.set_breakpoint('exploration')
        self.world_model = Thread(target=self.send_victim, args=(5,))
        self.target = mock_msgs.create_victim_info(id=1, probability=0.65)
        visited = [mock_msgs.create_victim_info() for i in range(0, 3)]
        self.agent.visited_victims = visited
        self.agent.current_victims.append(self.target)
        self.agent.target = self.target

    def send_victim(self, delay):
        """ Spawn a thread and send a potential victim instead of using a
            mock Publisher which is not so predictable or easy to configure.
        """
        victim = [mock_msgs.create_victim_info(id=1, probability=0.7)]
        visited = [mock_msgs.create_victim_info() for i in range(0, 3)]

        model = mock_msgs.create_world_model(victim, visited)
        sleep(delay)
        self.agent.receive_world_model(model)

    def test_delete_victim_success(self):
        self.delete_victim_mock.publish('success:1')
        self.agent.to_victim_deletion()

        self.assertEqual(self.agent.state, 'exploration')
        self.assertIsNone(self.agent.target)
        self.assertIn(self.target, self.agent.deleted_victims)

    def test_delete_victim_fail(self):
        self.delete_victim_mock.publish('abort:1')
        self.agent.to_victim_deletion()

        self.assertEqual(self.agent.state, 'exploration')
        self.assertIsNone(self.agent.target)
        self.assertIn(self.target, self.agent.deleted_victims)


class TestFusionValidationState(unittest.TestCase):
    """ Tests for the fusion validation state. """

    def setUp(self):
        self.agent = Agent(strategy='normal')
        self.fusion_validate = Publisher('mock/fusion_validate', String)
        self.target = mock_msgs.create_victim_info(id=1, probability=0.65)
        self.agent.set_breakpoint('exploration')
        self.agent.target = self.target

    def test_valid_victim(self):

        self.fusion_validate.publish('success:1')
        self.agent.gui_result.victimValid = True
        self.agent.to_fusion_validation()

        self.assertEqual(self.agent.state, 'exploration')
        self.assertFalse(self.agent.gui_result.victimValid)
        self.assertIsNone(self.agent.target)

    def test_invalid_victim(self):

        self.fusion_validate.publish('abort:1')
        self.agent.gui_result.victimValid = False
        self.agent.to_fusion_validation()

        self.assertEqual(self.agent.state, 'exploration')
        self.assertFalse(self.agent.gui_result.victimValid)
        self.assertIsNone(self.agent.target)


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
