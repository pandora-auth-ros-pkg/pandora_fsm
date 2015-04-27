#!/usr/bin/env python

"""
    Unit tests for the Agent class. The tests are made regardless of the
    state.
"""

import unittest
from threading import Event

import roslib
roslib.load_manifest('pandora_fsm')
import rospy
from rospy import Subscriber, Publisher, sleep
from std_msgs.msg import String, Int32

from actionlib import SimpleActionClient as Client
from actionlib_msgs.msg import GoalStatus

from pandora_fsm import Agent, TimeoutException, TimeLimiter, topics
from pandora_data_fusion_msgs.msg import WorldModelMsg


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

    @unittest.skip('Not ready yet.')
    def test_load(self):
        # TODO Write test with full functionality
        self.assertTrue(True)


class TestCallbacks(unittest.TestCase):
    """ Tests for the agent callbacks. """

    def setUp(self):
        self.agent = Agent(strategy='normal')
        self.world_model = Publisher('mock/world_model', Int32)

    def test_receive_world_model_response(self):
        self.world_model.publish(2)
        self.assertNotEqual(self.agent.current_victims, [])
        self.assertNotEqual(self.agent.visited_victims, [])


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


@unittest.skip('debug')
class TestExplorer(unittest.TestCase):
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


@unittest.skip('debug')
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

if __name__ == '__main__':
    rospy.init_node('test_agent_units')
    unittest.main()
