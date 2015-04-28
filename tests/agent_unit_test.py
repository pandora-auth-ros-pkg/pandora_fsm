#!/usr/bin/env python

"""
    Unit tests for the Agent class. The tests are made regardless of the
    state.
"""

import unittest
import threading

import roslib
roslib.load_manifest('pandora_fsm')
import rospy
from rospy import Subscriber, Publisher, sleep
from std_msgs.msg import String, Int32

from actionlib import SimpleActionClient as Client
from actionlib_msgs.msg import GoalStatus

from pandora_fsm import Agent, TimeoutException, TimeLimiter, topics
from pandora_data_fusion_msgs.msg import WorldModelMsg
import mock_publishers_subscribers

import mock_msgs
from pandora_fsm import MachineError


@unittest.skip('Save time.')
class TestROSIndependentMethods(unittest.TestCase):

    def setUp(self):
        """ Initialization """
        self.agent = Agent()

    def test_agent_initialization(self):

        self.assertEqual(self.agent.state, 'off')
        self.assertEqual(self.agent.valid_victims, 0)

        # Make sure the action clients are instantiated.
        self.assertIsInstance(self.agent.explorer, Client)
        self.assertIsInstance(self.agent.base_client, Client)
        self.assertIsInstance(self.agent.delete_victim_client, Client)
        self.assertIsInstance(self.agent.fusion_validate_client, Client)
        self.assertIsInstance(self.agent.gui_validate_client, Client)
        self.assertIsInstance(self.agent.end_effector_client, Client)
        self.assertIsInstance(self.agent.linear_client, Client)

        # Make sure the subscribers are instantiated.
        self.assertIsInstance(self.agent.arena_sub, Subscriber)
        self.assertIsInstance(self.agent.score_sub, Subscriber)
        self.assertIsInstance(self.agent.qr_sub, Subscriber)
        self.assertIsInstance(self.agent.area_coverage_sub, Subscriber)
        self.assertIsInstance(self.agent.world_model_sub, Subscriber)
        self.assertIsInstance(self.agent.linear_sub, Subscriber)

        # Make sure the threading.Events are initialized.
        self.assertIsInstance(self.agent.promising_victim, threading._Event)
        self.assertIsInstance(self.agent.accessible_victim, threading._Event)
        self.assertIsInstance(self.agent.recognized_victim, threading._Event)
        self.assertIsInstance(self.agent.explored, threading._Event)
        self.assertFalse(self.agent.promising_victim.is_set())
        self.assertFalse(self.agent.accessible_victim.is_set())
        self.assertFalse(self.agent.recognized_victim.is_set())

        # Empty variables
        self.assertEqual(self.agent.current_victims, [])
        self.assertEqual(self.agent.visited_victims, [])

    @unittest.skip('Not ready yet.')
    def test_load(self):
        # TODO Write test with full functionality
        self.assertTrue(True)


class TestWorldModelCallback(unittest.TestCase):
    """ Tests for the agent callbacks. """

    def setUp(self):
        self.agent = Agent(strategy='normal')
        self.world_model = Publisher('mock/world_model', String)

    @unittest.skip('Save time.')
    def test_receive_world_model_response(self):
        while not rospy.is_shutdown():
            self.world_model.publish('2')
            break
        sleep(3)
        self.assertNotEqual(self.agent.current_victims, [])
        self.assertNotEqual(self.agent.visited_victims, [])
        self.assertTrue(self.agent.promising_victim.is_set())

    def test_receive_world_model_with_target(self):
        """ Tests that the target is updated. """

        # Create a victim and assign it to the target.
        target = mock_msgs.create_victim_info()
        self.agent.target_victim = target

        self.assertEqual(self.agent.target_victim.id, target.id)

        # Create a custom world_model msg with updated target.
        target.probability = 0.8
        visited = [mock_msgs.create_victim_info() for i in range(2)]
        current_victims = [target]
        model = mock_msgs.create_world_model(current_victims, visited)

        self.agent.receive_world_model(model)
        self.assertEqual(self.agent.target_victim.id, target.id)
        self.assertEqual(self.agent.target_victim.probability, 0.8)
        self.assertTrue(self.agent.promising_victim.is_set())

    def test_receive_world_model_identification_threshold(self):
        """ Tests that the accessible_victim event is set. """

        self.agent.IDENTIFICATION_THRESHOLD = 0.5
        target = mock_msgs.create_victim_info(probability=0.7)
        self.agent.target_victim = target

        visited = [mock_msgs.create_victim_info() for i in range(2)]
        current_victims = [target]
        model = mock_msgs.create_world_model(current_victims, visited)

        self.agent.receive_world_model(model)
        self.assertEqual(self.agent.target_victim.id, target.id)
        self.assertTrue(self.agent.accessible_victim.is_set())

        target.probability = 0.2
        current_victims = [target]
        model = mock_msgs.create_world_model(current_victims, visited)

        self.agent.receive_world_model(model)
        self.assertEqual(self.agent.target_victim.id, target.id)
        self.assertFalse(self.agent.accessible_victim.is_set())

    def test_receive_world_model_verification_threshold(self):
        """ Tests that the recognized_victim event is set. """

        self.agent.VERIFICATION_THRESHOLD = 0.5
        target = mock_msgs.create_victim_info(probability=0.7)
        self.agent.target_victim = target

        visited = [mock_msgs.create_victim_info() for i in range(2)]
        current_victims = [target]
        model = mock_msgs.create_world_model(current_victims, visited)

        self.agent.receive_world_model(model)
        self.assertEqual(self.agent.target_victim.id, target.id)
        self.assertTrue(self.agent.recognized_victim.is_set())

        target.probability = 0.2
        current_victims = [target]
        model = mock_msgs.create_world_model(current_victims, visited)

        self.agent.receive_world_model(model)
        self.assertEqual(self.agent.target_victim.id, target.id)
        self.assertFalse(self.agent.recognized_victim.is_set())


@unittest.skip('Save time.')
class TestEndEffector(unittest.TestCase):
    """ Tests for the end effector action client. """

    def setUp(self):

        # Register the mock servers.
        self.effector_mock = Publisher('mock/effector', String)
        self.linear_mock = Publisher('mock/linear', String)
        self.agent = Agent(strategy='normal')

    def test_park_end_effector(self):

        self.effector_mock.publish(String('abort:1'))
        self.agent.park_end_effector_planner()
        self.assertEqual(self.agent.end_effector_client.get_state(),
                         GoalStatus.ABORTED)

        self.effector_mock.publish(String('success:1'))
        self.agent.park_end_effector_planner()
        self.assertEqual(self.agent.end_effector_client.get_state(),
                         GoalStatus.SUCCEEDED)

    def test_end_effector(self):

        self.effector_mock.publish(String('abort:1'))
        self.agent.test_end_effector_planner()
        self.assertEqual(self.agent.end_effector_client.get_state(),
                         GoalStatus.ABORTED)

        self.effector_mock.publish(String('success:1'))
        self.agent.test_end_effector_planner()
        self.assertEqual(self.agent.end_effector_client.get_state(),
                         GoalStatus.SUCCEEDED)

    def test_scan(self):
        self.effector_mock.publish(String('abort:1'))
        self.agent.scan()
        sleep(3)
        self.assertEqual(self.agent.end_effector_client.get_state(),
                         GoalStatus.ABORTED)

        self.effector_mock.publish(String('success:1'))
        self.agent.scan()
        sleep(3)
        self.assertEqual(self.agent.end_effector_client.get_state(),
                         GoalStatus.SUCCEEDED)

    def test_linear(self):
        self.linear_mock.publish(String('abort:1'))
        self.agent.test_linear_motor()
        self.assertEqual(self.agent.linear_client.get_state(),
                         GoalStatus.ABORTED)

        self.linear_mock.publish(String('success:1'))
        self.agent.test_linear_motor()
        self.assertEqual(self.agent.linear_client.get_state(),
                         GoalStatus.SUCCEEDED)


class TestMoveBase(unittest.TestCase):
    """ Tests for the base action client """

    def setUp(self):

        # Register the mock servers.
        self.move_base_mock = Publisher('mock/move_base', String)
        self.agent = Agent(strategy='normal')
        target = mock_msgs.create_victim_info(id=8, probability=0.65)
        self.agent.target_victim = target

    def test_move_base_abort(self):
        self.move_base_mock.publish(String('abort:1'))
        self.agent.move_base()
        self.agent.base_client.wait_for_result()
        self.assertEqual(self.agent.base_client.get_state(),
                         GoalStatus.ABORTED)

    def test_move_base_success(self):
        self.move_base_mock.publish(String('success:1'))
        self.agent.move_base()
        self.agent.base_client.wait_for_result()
        self.assertEqual(self.agent.base_client.get_state(),
                         GoalStatus.SUCCEEDED)

    def test_move_base_preempt(self):
        self.agent.move_base()
        self.agent.preempt_move_base()
        self.assertEqual(self.agent.base_client.get_state(),
                         GoalStatus.ABORTED)


class TestExplorer(unittest.TestCase):
    """ Tests for the explorer action client """

    def setUp(self):

        # Register the mock servers.
        self.explorer_mock = Publisher('mock/explorer', String)
        self.agent = Agent(strategy='normal')

    def test_preempt_explorer(self):
        self.explorer_mock.publish(String('abort:4'))
        self.agent.explore()
        self.assertEqual(self.agent.explorer.get_state(), GoalStatus.PENDING)
        self.agent.preempt_exploration()
        self.assertEqual(self.agent.explorer.get_state(), GoalStatus.ABORTED)
        self.assertFalse(self.agent.explored.is_set())

    def test_explorer_abort(self):
        self.explorer_mock.publish(String('abort:1'))
        self.agent.explore()
        sleep(3)
        self.assertEqual(self.agent.explorer.get_state(), GoalStatus.ABORTED)
        self.assertFalse(self.agent.explored.is_set())

    def test_explorer_success(self):
        self.explorer_mock.publish(String('success:1'))
        self.agent.explore()
        sleep(3)
        self.assertEqual(self.agent.explorer.get_state(), GoalStatus.SUCCEEDED)
        self.assertTrue(self.agent.explored.is_set())


class TestWaitForVictim(unittest.TestCase):
    """ Tests for the wait_for_victim_task. """

    def setUp(self):
        self.agent = Agent(strategy='normal')
        # Adding a fake state transition for the test, instead of
        # using the real FSM.
        self.agent.machine.add_state('test_victim_found')
        self.agent.machine.add_state('test_map_covered')
        self.agent.machine.add_transition('victim_found', 'off',
                                          'test_victim_found')
        self.agent.machine.add_transition('map_covered', 'off',
                                          'test_map_covered')

    def send_victim(self, delay):
        """ Spawn a thread and send a potential victim instead of using a
            mock Publisher which is not so predictable or easy to configure.
        """
        victim = [mock_msgs.create_victim_info()]
        visited = [mock_msgs.create_victim_info() for i in range(0, 3)]

        model = mock_msgs.create_world_model(victim, visited)
        sleep(delay)
        self.agent.receive_world_model(model)

    def set_map_covered(self, delay):
        """ Set the event that signals the end of the explorer. """

        sleep(delay)
        self.agent.explored.set()

    def test_victim_found(self):
        self.world_model = threading.Thread(target=self.send_victim, args=(1,))
        self.explorer = threading.Thread(target=self.set_map_covered,
                                         args=(4,))
        self.explorer.start()
        self.world_model.start()
        self.agent.wait_for_victim()

        self.assertEqual(self.agent.state, 'test_victim_found')

    def test_map_covered(self):
        self.world_model = threading.Thread(target=self.send_victim, args=(4,))
        self.explorer = threading.Thread(target=self.set_map_covered,
                                         args=(1,))
        self.explorer.start()
        self.world_model.start()
        self.agent.wait_for_victim()

        self.assertEqual(self.agent.state, 'test_map_covered')


class TestValidateGui(unittest.TestCase):

    def setUp(self):
        self.agent = Agent(strategy='normal')
        self.validate_gui_mock = Publisher('mock/validate_gui', String)
        self.set_gui_result = Publisher('mock/gui_result', String)

    def test_validated_true(self):
        """ The operator has stated this victim isvalid """
        self.agent.set_breakpoint('fusion_validation')
        msg = mock_msgs.create_victim_info(id=5)
        self.agent.target_victim = msg
        self.set_gui_result.publish(String('True'))
        self.validate_gui_mock.publish(String('success:2'))
        self.agent.to_operator_validation()
        self.assertEqual(self.agent.result.victimValid, True)

    def test_validated_false(self):
        """ The operator has stated this victim is not valid """
        self.agent.set_breakpoint('fusion_validation')
        msg = mock_msgs.create_victim_info(id=5)
        self.agent.target_victim = msg
        self.set_gui_result.publish(String('False'))
        self.validate_gui_mock.publish(String('success:2'))
        self.agent.to_operator_validation()
        self.assertEqual(self.agent.result.victimValid, False)

    def test_validation_aborted(self):
        """ It simulates the timeout """
        self.agent.set_breakpoint('fusion_validation')
        self.validate_gui_mock.publish(String('abort:2'))
        msg = mock_msgs.create_victim_info(id=5)
        self.agent.target_victim = msg
        self.agent.to_operator_validation()
        self.assertEqual(self.agent.gui_validate_client.get_state(),
                         GoalStatus.ABORTED)


class TestDeleteVictim(unittest.TestCase):

    def setUp(self):
        self.agent = Agent(strategy='normal')
        self.delete_victim_mock = Publisher('mock/delete_victim', String)
        target = mock_msgs.create_victim_info(id=8, probability=0.65)
        self.agent.target_victim = target

    # this function can't be testes completely autonomously because it triggers
    # victim_deleted
    def test_delete_victim(self):
        self.delete_victim_mock.publish('success:2')
        self.agent.set_breakpoint('exploration')
        self.agent.to_victim_deletion()
        self.assertEqual(self.agent.delete_victim_client.get_state(),
                         GoalStatus.SUCCEEDED)


if __name__ == '__main__':
    rospy.init_node('test_agent_units')
    unittest.main()
