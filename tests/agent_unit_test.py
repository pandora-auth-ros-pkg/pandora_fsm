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
from std_msgs.msg import String

from actionlib import SimpleActionClient as Client
from actionlib_msgs.msg import GoalStatus

from pandora_fsm import Agent, TimeoutException, TimeLimiter, topics
from pandora_data_fusion_msgs.msg import WorldModelMsg
import mock


# @unittest.skip('save time')
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

    def test_load(self):
        # TODO Write test with full functionality
        self.assertTrue(True)


# @unittest.skip('save time')
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


@unittest.skip('Not ready yet.')
class TestMoveBase(unittest.TestCase):
    """ Tests for the base action client """

    def setUp(self):

        # Register the mock servers.
        self.move_base_mock = Publisher('mock/move_base', String)
        self.agent = Agent(strategy='normal')

    def test_move_base(self):
        self.move_base_mock.publish(String('abort:1'))
        self.agent.move_base()
        self.assertEqual(self.agent.base_client.get_state(),
                         GoalStatus.ABORTED)

        self.move_base_mock.publish(String('success:1'))
        self.agent.move_base()
        self.assertEqual(self.agent.base_client.get_state(),
                         GoalStatus.SUCCEEDED)


# @unittest.skip('save time.')
class TestExploration(unittest.TestCase):
    """ Tests for the explorer action client """

    def setUp(self):

        # Register the mock servers.
        self.explorer_mock = Publisher('mock/explorer', String)
        self.victim_mock = Publisher(topics.world_model, WorldModelMsg)
        self.agent = Agent(strategy='normal')

    def test_explore(self):
        self.explorer_mock.publish(String('abort:1'))
        self.agent.explore()
        sleep(5.0)
        self.assertEqual(self.agent.explorer.get_state(),
                         GoalStatus.ABORTED)

        self.explorer_mock.publish(String('success:1'))
        self.agent.explore()
        sleep(5.0)
        self.assertEqual(self.agent.explorer.get_state(),
                         GoalStatus.SUCCEEDED)


# @unittest.skip('save time.')
class TestUpdateVictim(unittest.TestCase):
    def setUp(self):
        self.my_world = Publisher('mock/victim_probability', String)
        self.agent = Agent(strategy='normal')

    def test_update(self):

        msg = mock.create_victim_info(id=8, probability=0.65)
        self.agent.target_victim = msg
        sleep(2)
        self.assertAlmostEqual(self.agent.target_victim.probability, 0.65)
        self.my_world.publish('8:0.88')
        sleep(2)
        self.agent.update_target_victim()
        self.assertAlmostEqual(self.agent.target_victim.probability, 0.88)
        sleep(8.0)
        self.my_world.publish('8:0.95')
        sleep(2)
        self.agent.update_target_victim()
        self.assertAlmostEqual(self.agent.target_victim.probability, 0.95)


# @unittest.skip('save time.')
class TestInitState(unittest.TestCase):
    """ Tests for the agent's tasks. A task is a function that is executed
        within a state. A task can belong to multiple states.
    """

    def setUp(self):

        # Register the mock servers.
        self.effector_mock = Publisher('mock/effector', String)
        self.linear_mock = Publisher('mock/linear', String)
        self.explorer_mock = Publisher('mock/explorer', String)
        self.move_base_mock = Publisher('mock/move', String)
        self.world_model = Publisher('mock/world_model', String)
        self.validate_gui_mock = Publisher('mock/validate_gui', String)
        self.my_world = Publisher('mock/victim_probability', String)
        self.delete_victim_mock = Publisher('/mock/delete_victim', String)
        self.agent = Agent(strategy='normal')

    # @unittest.skip('save time')
    def test_initialization_from_sleep(self):

        self.effector_mock.publish(String('success:1'))
        self.linear_mock.publish(String('success:1'))
        self.agent.set_breakpoint('exploration')
        self.agent.wake_up()
        self.assertEqual(self.agent.state, 'exploration')

    @unittest.skip('Not working exploration needs fixing')
    def test_initialization_to_end(self):
        self.effector_mock.publish(String('success:1'))
        self.linear_mock.publish(String('success:1'))
        self.explorer_mock.publish(String('success:1'))
        self.agent.set_breakpoint('end')
        self.agent.wake_up()
        sleep(2.0)
        self.assertEqual(self.agent.state, 'end')

    # @unittest.skip('save time')
    def test_initialization_to_closeup(self):
        self.effector_mock.publish(String('success:1'))
        self.linear_mock.publish(String('success:1'))
        self.move_base_mock.publish(String('success:1'))
        self.explorer_mock.publish(String('abort:1'))
        self.world_model.publish('0.2')
        self.agent.set_breakpoint('closeup')
        self.agent.wake_up()
        self.assertEqual(self.agent.state, 'closeup')

    # @unittest.skip('save time')
    def test_initialization_to_victim_deletion(self):
        self.effector_mock.publish(String('success:1'))
        self.linear_mock.publish(String('success:1'))
        self.explorer_mock.publish(String('abort:1'))
        self.move_base_mock.publish(String('abort:1'))
        self.my_world.publish('2:0.4')
        self.agent.set_breakpoint('victim_deletion')
        self.agent.wake_up()
        self.assertEqual(self.agent.state, 'victim_deletion')

    # @unittest.skip('save time')
    def test_initialization_to_fusion_validation(self):
        self.effector_mock.publish(String('success:1'))
        self.linear_mock.publish(String('success:1'))
        self.move_base_mock.publish(String('success:1'))
        self.explorer_mock.publish(String('abort:1'))
        self.my_world.publish('2:0.4')
        self.agent.set_breakpoint('fusion_validation')
        self.agent.wake_up()
        self.assertEqual(self.agent.state, 'fusion_validation')

    # @unittest.skip('save time')
    def test_initialization_to_operator_validation(self):
        self.effector_mock.publish(String('success:1'))
        self.linear_mock.publish(String('success:1'))
        self.move_base_mock.publish(String('success:1'))
        self.explorer_mock.publish(String('abort:1'))
        self.my_world.publish('2:0.9')
        # self.my_world.publish_custom_msg(probability=0.9)
        self.agent.set_breakpoint('operator_validation')
        self.agent.wake_up()
        self.assertEqual(self.agent.state, 'operator_validation')

    # @unittest.skip('save time')
    def test_initialization_to_operator_with_aborted_move_base(self):
        self.effector_mock.publish(String('success:1'))
        self.linear_mock.publish(String('success:1'))
        self.move_base_mock.publish(String('abort:1'))
        self.explorer_mock.publish(String('abort:1'))
        self.my_world.publish('2:0.9')
        # self.my_world.publish_custom_msg(probability=0.9)
        self.agent.set_breakpoint('operator_validation')
        self.agent.wake_up()
        self.assertEqual(self.agent.state, 'operator_validation')

    # @unittest.skip('save time')
    def test_initialization_to_fusion_validation_through_operator_1(self):
        # operator goal aborted
        self.effector_mock.publish(String('success:1'))
        self.linear_mock.publish(String('success:1'))
        self.move_base_mock.publish(String('abort:1'))
        self.explorer_mock.publish(String('abort:1'))
        self.validate_gui_mock.publish(String('abort:1'))
        self.my_world.publish('2:0.9')
        self.agent.set_breakpoint('fusion_validation')
        self.agent.wake_up()
        self.assertEqual(self.agent.state, 'fusion_validation')

    # @unittest.skip('save time')
    def test_initialization_to_fusion_validation_through_operator_2(self):
        # operator goal succeeded
        self.effector_mock.publish(String('success:1'))
        self.linear_mock.publish(String('success:1'))
        self.move_base_mock.publish(String('abort:1'))
        self.explorer_mock.publish(String('abort:1'))
        self.validate_gui_mock.publish(String('success:1'))
        self.my_world.publish('2:0.9')
        self.agent.set_breakpoint('fusion_validation')
        self.agent.wake_up()
        self.assertEqual(self.agent.state, 'fusion_validation')

    # @unittest.skip('save_time')
    def initialization_to_permanent_exploration(self):

        self.effector_mock.publish(String('success:1'))
        self.linear_mock.publish(String('success:1'))
        self.explorer_mock.publish(String('abort:1'))
        self.agent.set_breakpoint('exploration')
        self.agent.wake_up()
        self.assertEqual(self.agent.state, 'exploration')

    # @unittest.skip('save time')
    def test_immediate_initialization_with_linear_failure(self):

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

    # @unittest.skip('save time')
    def test_immediate_initialization_with_effector_failure(self):

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


if __name__ == '__main__':
    rospy.init_node('test_node')
    unittest.main()
