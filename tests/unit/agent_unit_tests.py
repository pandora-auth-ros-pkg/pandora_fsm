#!/usr/bin/env python

"""
    Unit tests for the Agent class. The tests are made regardless of the
    state.
"""

import unittest

import roslib
roslib.load_manifest('pandora_fsm')
import rospy
from rospy import Subscriber, Publisher, sleep
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped

from actionlib_msgs.msg import GoalStatus

from pandora_fsm.mocks import msgs as mock_msgs
from pandora_fsm.utils import distance_2d, distance_3d
from pandora_fsm import (Agent, TimeLimiter, TimeoutException, Navigation,
                         Control, DataFusion, GUI, Effector)


class TestUtils(unittest.TestCase):

    def setUp(self):
        """ Initialization """
        self.agent = Agent()

    def test_agent_initialization(self):

        self.assertEqual(self.agent.state, 'off')
        self.assertEqual(self.agent.victims_found, 0)

        # Make sure the action clients are instantiated.
        self.assertIsInstance(self.agent.explorer, Navigation)
        self.assertIsInstance(self.agent.control_base, Control)
        self.assertIsInstance(self.agent.data_fusion, DataFusion)
        self.assertIsInstance(self.agent.gui_client, GUI)
        self.assertIsInstance(self.agent.effector, Effector)

        # Make sure the subscribers are instantiated.
        self.assertIsInstance(self.agent.score_sub, Subscriber)
        self.assertIsInstance(self.agent.qr_sub, Subscriber)
        self.assertIsInstance(self.agent.world_model_sub, Subscriber)

        # Make sure global state transition functios have been generated.
        self.assertIsNotNone(self.agent.mode_off)
        self.assertIsNotNone(self.agent.mode_start_autonomous)
        self.assertIsNotNone(self.agent.mode_exploration_rescue)
        self.assertIsNotNone(self.agent.mode_identification)
        self.assertIsNotNone(self.agent.mode_sensor_hold)
        self.assertIsNotNone(self.agent.mode_semi_autonomous)
        self.assertIsNotNone(self.agent.mode_teleoperated_locomotion)
        self.assertIsNotNone(self.agent.mode_sensor_test)
        self.assertIsNotNone(self.agent.mode_exploration_mapping)
        self.assertIsNotNone(self.agent.mode_terminating)

        # Empty variables
        self.assertEqual(self.agent.current_victims, [])
        self.assertEqual(self.agent.visited_victims, [])

    @unittest.skip('Not ready yet.')
    def test_load(self):
        # TODO Write test with full functionality
        self.assertTrue(True)

    def test_distance_2d(self):
        a = PoseStamped()
        b = PoseStamped()
        a.pose.position.x = -7
        a.pose.position.y = -4
        b.pose.position.x = 17
        b.pose.position.y = 6.5

        self.assertAlmostEqual(distance_2d(a.pose, b.pose), 26.19637379)

    def test_distance_3d(self):
        a = PoseStamped()
        b = PoseStamped()

        a.pose.position.x = -7
        a.pose.position.y = -4
        a.pose.position.z = 3

        b.pose.position.x = 17
        b.pose.position.y = 6
        b.pose.position.z = 2.5

        self.assertAlmostEqual(distance_3d(a.pose, b.pose), 26.0048072)


class TestWorldModelCallback(unittest.TestCase):
    """ Tests for the agent callbacks. """

    def setUp(self):
        self.agent = Agent(strategy='normal')
        self.world_model = Publisher('mock/world_model', String)

    def test_receive_world_model_response(self):
        # To assert point_of_interest
        self.agent.state = 'exploration'
        while not rospy.is_shutdown():
            self.world_model.publish('1')
            break
        sleep(3)
        self.assertNotEqual(self.agent.current_victims, [])
        self.assertNotEqual(self.agent.visited_victims, [])
        self.assertTrue(self.agent.point_of_interest.is_set())

    def test_receive_world_model_with_target(self):
        """ Tests that the target is updated. """

        # To assert point_of_interest
        self.agent.state = 'exploration'

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
        self.assertTrue(self.agent.point_of_interest.is_set())

    def test_receive_world_model_identification_threshold(self):
        """ Tests that the promising_victim event is set. """

        # To assert point_of_interest
        self.agent.state = 'exploration'
        self.agent.IDENTIFICATION_THRESHOLD = 0.5
        target = mock_msgs.create_victim_info(probability=0.7)
        self.agent.target_victim = target

        visited = [mock_msgs.create_victim_info() for i in range(2)]
        current_victims = [target]
        model = mock_msgs.create_world_model(current_victims, visited)

        self.agent.receive_world_model(model)
        self.assertEqual(self.agent.target_victim.id, target.id)
        self.assertTrue(self.agent.promising_victim.is_set())

        target.probability = 0.2
        current_victims = [target]
        model = mock_msgs.create_world_model(current_victims, visited)

        self.agent.receive_world_model(model)
        self.assertEqual(self.agent.target_victim.id, target.id)
        self.assertFalse(self.agent.promising_victim.is_set())

    def test_receive_world_model_verification_threshold(self):
        """ Tests that the recognized_victim event is set. """

        # To assert point_of_interest
        self.agent.state = 'exploration'
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


class TestEndEffector(unittest.TestCase):
    """ Tests for the end effector action client. """

    def setUp(self):

        # Register the mock servers.
        self.effector_mock = Publisher('mock/effector', String)
        self.agent = Agent(strategy='normal')

    def test_park_end_effector(self):

        self.effector_mock.publish('abort:1')

        @TimeLimiter(timeout=5)
        def infinite_delay():
            self.agent.park_end_effector()

        self.assertRaises(TimeoutException, infinite_delay)

        self.effector_mock.publish('success:1')
        self.agent.park_end_effector()
        self.assertEqual(self.agent.end_effector_client.get_state(),
                         GoalStatus.SUCCEEDED)

    def test_end_effector(self):

        @TimeLimiter(timeout=5)
        def infinite_delay():
            self.agent.test_end_effector()

        self.assertRaises(TimeoutException, infinite_delay)

        self.effector_mock.publish('success:1')
        self.agent.test_end_effector()
        self.assertEqual(self.agent.end_effector_client.get_state(),
                         GoalStatus.SUCCEEDED)

    def test_scan(self):
        self.effector_mock.publish('abort:1')
        self.agent.scan()
        sleep(3)
        self.assertEqual(self.agent.end_effector_client.get_state(),
                         GoalStatus.ABORTED)

        self.effector_mock.publish('success:1')
        self.agent.scan()
        sleep(3)
        self.assertEqual(self.agent.end_effector_client.get_state(),
                         GoalStatus.SUCCEEDED)

    def test_point_sensors(self):
        self.agent.target_victim = mock_msgs.create_victim_info()
        self.effector_mock.publish('abort:1')
        self.agent.point_sensors()
        sleep(3)
        self.assertEqual(self.agent.end_effector_client.get_state(),
                         GoalStatus.ABORTED)

        self.effector_mock.publish('success:1')
        self.agent.point_sensors()
        sleep(3)
        self.assertEqual(self.agent.end_effector_client.get_state(),
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
        self.move_base_mock.publish('abort:1')
        self.agent.move_base()
        self.agent.control_base.wait_for_result()
        self.assertEqual(self.agent.control_base.get_state(),
                         GoalStatus.ABORTED)

    def test_move_base_success(self):
        self.move_base_mock.publish('success:1')
        self.agent.move_base()
        self.agent.control_base.wait_for_result()
        self.assertEqual(self.agent.control_base.get_state(),
                         GoalStatus.SUCCEEDED)

    def test_move_base_preempt(self):
        self.agent.move_base()
        self.agent.preempt_move_base()
        self.assertEqual(self.agent.control_base.get_state(),
                         GoalStatus.ABORTED)


class TestExplorer(unittest.TestCase):
    """ Tests for the explorer action client """

    def setUp(self):

        # Register the mock servers.
        self.explorer_mock = Publisher('mock/explorer', String)
        self.agent = Agent(strategy='normal')

    def test_preempt_explorer(self):
        self.explorer_mock.publish('abort:4')
        self.agent.explore()
        self.assertEqual(self.agent.explorer.get_state(), GoalStatus.PENDING)
        self.agent.preempt_exploration()
        self.assertEqual(self.agent.explorer.get_state(), GoalStatus.ABORTED)
        self.assertFalse(self.agent.exploration_done.is_set())

    def test_explorer_abort(self):
        self.explorer_mock.publish('abort:1')
        self.agent.explore()
        sleep(3)
        self.assertEqual(self.agent.explorer.get_state(), GoalStatus.ABORTED)
        self.assertFalse(self.agent.exploration_done.is_set())

    def test_explorer_success(self):
        self.explorer_mock.publish('success:1')
        self.agent.explore()
        sleep(3)
        self.assertEqual(self.agent.explorer.get_state(), GoalStatus.SUCCEEDED)
        self.assertTrue(self.agent.exploration_done.is_set())


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


class TestDeleteVictim(unittest.TestCase):

    def setUp(self):
        self.agent = Agent(strategy='normal')
        self.agent.machine.add_state('test_delete_victim')
        self.agent.machine.add_transition('victim_deleted', 'off',
                                          'test_delete_victim')
        self.delete_victim_mock = Publisher('mock/delete_victim', String)
        self.agent.target_victim = mock_msgs.create_victim_info()

    def test_delete_victim_with_abort(self):
        """ If the goal is aborted the agent will keep trying. """

        while not rospy.is_shutdown():
            sleep(2)
            self.delete_victim_mock.publish('abort:1')
            break

        @TimeLimiter(timeout=7)
        def infinite_delay():
            self.agent.delete_victim()

        self.assertRaises(TimeoutException, infinite_delay)
        self.assertEqual(self.agent.state, 'off')

    def test_delete_victim_success(self):

        while not rospy.is_shutdown():
            sleep(2)
            self.delete_victim_mock.publish('success:1')
            break

        self.agent.delete_victim()

        self.assertEqual(self.agent.state, 'test_delete_victim')
        self.assertEqual(self.agent.data_fusion.get_state(),
                         GoalStatus.SUCCEEDED)


if __name__ == '__main__':
    rospy.init_node('unit_tests')
    unittest.main()
