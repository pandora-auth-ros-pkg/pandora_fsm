#!/usr/bin/env python

"""
    Unit tests for the Agent class. The tests are made regardless of the
    state.
"""

import unittest
from threading import Thread

import rostest

import rospy
from rospy import Subscriber

from actionlib import SimpleActionClient as Client

from pandora_fsm import Agent
from pandora_behave import MockActionServer

from pandora_end_effector_planner.msg import MoveEndEffectorAction

class TestROSIndependentMethods(unittest.TestCase):

    def setUp(self):
        """ Initialization """
        self.woody = Agent()

    def test_agent_initialization(self):

        self.assertEqual(self.woody.state, 'off')
        self.assertEqual(self.woody.valid_victims, 0)

        # Make sure the action clients are instantiated.
        self.assertIsInstance(self.woody.explorer, Client)
        self.assertIsInstance(self.woody.base_client, Client)
        self.assertIsInstance(self.woody.delete_victim_client, Client)
        self.assertIsInstance(self.woody.fusion_validate_client, Client)
        self.assertIsInstance(self.woody.gui_validate_client, Client)
        self.assertIsInstance(self.woody.end_effector_client, Client)

        # Make sure the subscribers are instantiated.
        self.assertIsInstance(self.woody.arena_sub, Subscriber)
        self.assertIsInstance(self.woody.score_sub, Subscriber)
        self.assertIsInstance(self.woody.qr_sub, Subscriber)
        self.assertIsInstance(self.woody.area_coverage_sub, Subscriber)
        self.assertIsInstance(self.woody.world_model_sub, Subscriber)
        self.assertIsInstance(self.woody.linear_sub, Subscriber)


class TestAgentActions(unittest.TestCase):
    """ Tests for the agent's methods used as tasks. """

    def setUp(self):
        """ """
        self.woody = Agent()

    def test_park_end_effector_planner(self):

        # Create a mock for the action server.
        end_effector_mock = MockActionServer('test', MoveEndEffectorAction)
        mock = Thread(target=end_effector_mock.run)

        self.woody.park_end_effector_planner()

if __name__ == '__main__':
    rospy.init_node('unit_tests')
    # rostest.rosrun('pandora_fsm', 'test_fsm', TestROSIndependentMethods)
    # rostest.rosrun('pandora_fsm', 'test_fsm', TestAgentActions)
