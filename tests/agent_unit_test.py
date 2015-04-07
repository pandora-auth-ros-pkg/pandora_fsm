#!/usr/bin/env python

"""
    Unit tests for the Agent class. The tests are made regardless of the
    state.
"""

import unittest
from threading import Thread

import rostest
import roslib
roslib.load_manifest('pandora_fsm')
import rospy
from rospy import Subscriber, Publisher

from std_msgs.msg import String

from actionlib import SimpleActionClient as Client
from actionlib_msgs.msg import GoalStatus

from mock import MockEndEffector

from pandora_fsm import Agent, topics
from pandora_behave import MockActionServer

from pandora_end_effector_planner.msg import MoveEndEffectorAction
from pandora_end_effector_planner.msg import MoveEndEffectorGoal


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

        # Make sure the subscribers are instantiated.
        self.assertIsInstance(self.agent.arena_sub, Subscriber)
        self.assertIsInstance(self.agent.score_sub, Subscriber)
        self.assertIsInstance(self.agent.qr_sub, Subscriber)
        self.assertIsInstance(self.agent.area_coverage_sub, Subscriber)
        self.assertIsInstance(self.agent.world_model_sub, Subscriber)
        self.assertIsInstance(self.agent.linear_sub, Subscriber)

    def test_load(self):
        self.assertTrue(True)


class TestEndEffector(unittest.TestCase):
    """ Tests for the end effector action client. """

    def setUp(self):

        # Register the mock servers.
        self.cmd_pub = Publisher('mock/cmd', String)
        self.agent = Agent(config='testing.json', strategy='test_init')

    def test_park_end_effector(self):

        self.cmd_pub.publish(String('ABORTED'))
        self.agent.park_end_effector_planner()
        self.assertEqual(self.agent.end_effector_client.get_state(),
                         GoalStatus.ABORTED)

        self.cmd_pub.publish(String('SUCCEEDED'))
        self.agent.park_end_effector_planner()
        self.assertEqual(self.agent.end_effector_client.get_state(),
                         GoalStatus.SUCCEEDED)

    def test_end_effector(self):

        self.cmd_pub.publish(String('ABORTED'))
        self.agent.test_end_effector_planner()
        self.assertEqual(self.agent.end_effector_client.get_state(),
                         GoalStatus.ABORTED)

        self.cmd_pub.publish(String('SUCCEEDED'))
        self.agent.test_end_effector_planner()
        self.assertEqual(self.agent.end_effector_client.get_state(),
                         GoalStatus.SUCCEEDED)


class TestInitState(unittest.TestCase):
    """ Tests for the agent's tasks. A task is a function that is executed
        within a state. A task can belong to multiple states.
    """

    def setUp(self):

        # Register the mock servers.
        self.cmd_pub = Publisher('mock/cmd', String)
        self.agent = Agent(config='testing.json', strategy='test_init')

    def test_initialization_from_sleep(self):

        self.agent.wake_up()
        self.assertEqual(self.agent.state, 'exploration')

    def test_immediate_initialization(self):

        # FIXME Add a signal to make the test fail
        # when it goes in an infinite loop.
        self.cmd_pub.publish(String('ABORTED'))

        # It will hang indefinitely
        self.agent.to_init()
        self.assertEqual(self.agent.state, 'exploration')

if __name__ == '__main__':
    rospy.init_node('test_node')
    unittest.main()
