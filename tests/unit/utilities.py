#!/usr/bin/env python

"""
    Unit Tests for utility methods.
"""

import unittest

import roslib
roslib.load_manifest('pandora_fsm')
import rospy
from rospy import Subscriber
from geometry_msgs.msg import PoseStamped


from pandora_fsm.utils import distance_2d, distance_3d
from pandora_fsm import Agent, Navigation, Control, DataFusion, GUI, Effector


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


if __name__ == '__main__':
    rospy.init_node('unit_utility')
    unittest.main()
