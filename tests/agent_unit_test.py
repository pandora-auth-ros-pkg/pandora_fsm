#!/usr/bin/env python

"""
    Unit tests for the Agent class. The tests are made regardless of the
    state.
"""

import unittest

import roslib
roslib.load_manifest('pandora_fsm')
import rospy
from rospy import Subscriber, Publisher

from std_msgs.msg import String

from actionlib import SimpleActionClient as Client
from actionlib_msgs.msg import GoalStatus

from pandora_fsm import Agent, TimeoutException, TimeLimiter


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

    @unittest.skip('Not ready yet.')
    def test_scan(self):
        self.cmd_pub.publish(String('ABORTED'))
        self.agent.scan()
        self.assertEqual(self.agent.end_effector_client.get_state(),
                         GoalStatus.ABORTED)

        self.cmd_pub.publish(String('SUCCEEDED'))
        self.agent.scan()
        self.assertEqual(self.agent.end_effector_client.get_state(),
                         GoalStatus.SUCCEDED)

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
        self.cmd_pub = Publisher('mock/cmd', String)
        self.agent = Agent(strategy='normal')

    def test_move_base(self):
        self.cmd_pub.publish(String('ABORTED'))
        self.agent.move_base()
        self.assertEqual(self.agent.base_client.get_state(),
                         GoalStatus.ABORTED)

        self.cmd_pub.publish(String('SUCCEEDED'))
        self.agent.move_base()
        self.assertEqual(self.agent.base_client.get_state(),
                         GoalStatus.SUCCEDED)


@unittest.skip('Not ready yet.')
class TestExploration(unittest.TestCase):
    """ Tests for the explorer action client """

    def setUp(self):

        # Register the mock servers.
        self.cmd_pub = Publisher('mock/cmd', String)
        self.agent = Agent(strategy='normal')

    def test_explore(self):
        self.cmd_pub.publish(String('ABORTED'))
        self.agent.explore()
        self.assertEqual(self.agent.explorer.get_state(),
                         GoalStatus.ABORTED)

        self.cmd_pub.publish(String('SUCCEEDED'))
        self.agent.explore()
        self.assertEqual(self.agent.explorer.get_state(),
                         GoalStatus.SUCCEDED)


@unittest.skip('Not ready yet.')
class TestFusionValidation(unittest.TestCase):
    """ Tests for the fusion validation action client """

    def setUp(self):

        # Register the mock servers.
        self.cmd_pub = Publisher('mock/cmd', String)
        self.agent = Agent(strategy='normal')

    def test_victim_classification(self):
        self.cmd_pub.publish(String('ABORTED'))
        self.agent.victim_classification()
        self.assertEqual(self.fusion_validate_client.get_state(),
                         GoalStatus.ABORTED)

        self.cmd_pub.publish(String('SUCCEEDED'))
        self.agent.victim_classification
        self.assertEqual(self.fusion_validate_client.get_state(),
                         GoalStatus.SUCCEDED)


@unittest.skip('Not ready yet.')
class TestGuiValidationClient(unittest.TestCase):
    """ Tests for the GUI validation action client """

    def setUp(self):

        # Register the mock servers.
        self.cmd_pub = Publisher('mock/cmd', String)
        self.agent = Agent(strategy='normal')

    def test_operator_confirmation(self):
        self.cmd_pub.publish(String('ABORTED'))
        self.agent.operator_confirmation()
        self.assertEqual(self.gui_validate_client.get_state(),
                         GoalStatus.ABORTED)

        self.cmd_pub.publish(String('SUCCEEDED'))
        self.agent.operator_confirmation()
        self.assertEqual(self.gui_validate_client.get_state(),
                         GoalStatus.SUCCEEDED)


class TestInitState(unittest.TestCase):
    """ Tests for the agent's tasks. A task is a function that is executed
        within a state. A task can belong to multiple states.
    """

    def setUp(self):

        # Register the mock servers.
        self.effector_mock = Publisher('mock/effector', String)
        self.linear_mock = Publisher('mock/linear', String)
        self.agent = Agent(strategy='normal')

    def test_initialization_from_sleep(self):

        self.effector_mock.publish(String('success:1'))
        self.linear_mock.publish(String('success:1'))
        self.agent.set_breakpoint('exploration')
        self.agent.wake_up()
        self.assertEqual(self.agent.state, 'exploration')

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
