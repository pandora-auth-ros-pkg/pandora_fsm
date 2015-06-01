#!/usr/bin/env python

"""
    Unit Tests for utility methods.
"""

import threading
import time
import unittest

from mock import patch
import roslib
roslib.load_manifest('pandora_fsm')

from rospy import Subscriber
from geometry_msgs.msg import PoseStamped

from pandora_fsm.utils import distance_2d, distance_3d, Timer
from pandora_fsm import Agent, Explorer, Navigator, DataFusion, GUI, Effector


class TestUtils(unittest.TestCase):

    def setUp(self):
        """ Initialization """
        self.agent = Agent(testing=True)

    def test_agent_initialization(self):

        self.assertEqual(self.agent.state, 'off')

        # Make sure the action clients are instantiated.
        self.assertIsInstance(self.agent.explorer, Explorer)
        self.assertIsInstance(self.agent.navigator, Navigator)
        self.assertIsInstance(self.agent.data_fusion, DataFusion)
        self.assertIsInstance(self.agent.gui_client, GUI)
        self.assertIsInstance(self.agent.effector, Effector)

        # Make sure the subscribers are instantiated.
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
        self.assertEqual(self.agent.available_targets, [])

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


class ThreadTimer(unittest.TestCase):

    def setUp(self):
        self.flag = threading.Event()

    def callback(self):
        pass

    def callback_kwargs(self, arg, a=1, b=2):
        pass

    def test_simple_call(self):
        """ The callback should be called in the simplest occasion. """

        with patch.object(self, 'callback') as mock:
            Timer(self.flag, 1, self.callback).start()
            time.sleep(2.5)
            self.flag.set()
            self.assertEqual(mock.call_count, 2)

    def test_no_calls(self):
        """ The callback should not be executed if the flag is set. """

        with patch.object(self, 'callback') as mock:
            self.flag.set()
            Timer(self.flag, 1, self.callback).start()
            time.sleep(2)
            self.assertEqual(mock.call_count, 0)

    def test_restart(self):
        """ The timer should start again after a stop. """

        with patch.object(self, 'callback') as mock:
            Timer(self.flag, 1, self.callback).start()
            time.sleep(2.5)
            self.flag.set()
            self.assertEqual(mock.call_count, 2)

            self.flag.clear()
            Timer(self.flag, 1, self.callback).start()
            time.sleep(2.5)
            self.flag.set()
            self.assertGreater(mock.call_count, 4)

    def test_passing_arguments(self):
        """ The callback should accept args and kwargs. """

        with patch.object(self, 'callback_kwargs') as mock:
            args = (3, )
            Timer(self.flag, 1, self.callback_kwargs, args, a=2, b=7).start()
            time.sleep(2.5)
            self.flag.set()
            mock.assert_called_with(3, a=2, b=7)
