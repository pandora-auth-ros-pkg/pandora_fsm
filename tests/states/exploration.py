#!/usr/bin/env python

import unittest
from threading import Thread

from mock import patch

import rospy
import roslib
roslib.load_manifest('pandora_fsm')

from rospy import Publisher, sleep
from std_msgs.msg import String

from state_manager_msgs.msg import RobotModeMsg
from pandora_fsm import Agent
from pandora_fsm.clients import Effector

from pandora_fsm.mocks import msgs as mock_msgs


class TestExplorationState(unittest.TestCase):
    """ Tests for the exploration state. """

    def setUp(self):
        rospy.init_node('state_epxloration_test')
        self.agent = Agent(strategy='normal')
        self.agent.set_breakpoint('identification')
        self.agent.set_breakpoint('end')
        self.agent.set_breakpoint('init')
        self.explorer = Publisher('mock/explorer', String)
        self.agent.preempt_end_effector = lambda: True
        self.agent.preempt_explorer = lambda: True

    def random_victims(self, times, delay):
        """ Spawn a thread and send a potential victim instead of using a
            mock Publisher which is not so predictable or easy to configure.
        """
        visited = [mock_msgs.create_victim_info() for i in range(0, 3)]
        victim = [mock_msgs.create_victim_info()]

        model = mock_msgs.create_world_model(victim, visited)
        for i in range(times):
            sleep(delay)
            self.agent.receive_world_model(model)

    def test_target_found_before_exploration(self):
        """ A target has been discovered before going to exploration. """

        # Long goals that will not affect the test.
        self.agent.disable_events()
        if not rospy.is_shutdown():
            sleep(1)
            self.explorer.publish('success:20')
        Thread(target=self.random_victims, args=(5, 1,)).start()
        sleep(2)
        with patch.object(self.agent, 'receive_world_model') as mock:
            self.agent.to_exploration()
            sleep(5)
            self.assertGreaterEqual(mock.call_count, 3)
            self.assertFalse(self.agent.target.is_empty)
            self.assertEqual(self.agent.state, 'identification')

    def test_target_found_after_exploration(self):
        """ A target has been discovered after exploration. """

        self.agent.disable_events()
        if not rospy.is_shutdown():
            sleep(1)
            self.explorer.publish('success:20')
        Thread(target=self.random_victims, args=(3, 2,)).start()
        self.agent.to_exploration()
        sleep(5)
        self.assertFalse(self.agent.target.is_empty)
        self.assertEqual(self.agent.state, 'identification')

    def test_to_end(self):

        # This goal will move the agent to the end state.
        if not rospy.is_shutdown():
            sleep(1)
            self.explorer.publish('success:1')
        self.agent.to_exploration()
        sleep(5)
        self.assertEqual(self.agent.state, 'end')

    def test_long_wait_for_victim(self):
        """ The agent will wait for a victim indefinitely. """

        # Long goals that will not affect the test.
        if not rospy.is_shutdown():
            sleep(1)
            self.explorer.publish('success:20')
        self.agent.to_exploration()
        sleep(10)
        self.assertEqual(self.agent.state, 'exploration')
        self.assertTrue(self.agent.target.is_empty)

    def test_retry_on_explorer_abort(self):
        """ The agent will keep sending goals if the explorer fails. """

        if not rospy.is_shutdown():
            sleep(1)
            self.explorer.publish('abort:1')

        with patch.object(self.agent.explorer.dispatcher, 'emit') as mock:
            self.agent.to_exploration()
            sleep(7)
        mock.assert_called_with('exploration.retry')
        self.assertEqual(self.agent.state, 'exploration')

    def test_retry_on_explorer_reject(self):
        """ The agent will keep sending goals if the explorer fails. """

        if not rospy.is_shutdown():
            sleep(1)
            self.explorer.publish('reject:1')

        with patch.object(self.agent.explorer.dispatcher, 'emit') as mock:
            self.agent.to_exploration()
            sleep(7)
        mock.assert_called_with('exploration.retry')
        self.assertEqual(self.agent.state, 'exploration')

    def test_global_state_change(self):
        """ The global state should be MODE_EXPLORATION_RESCUE """

        if not rospy.is_shutdown():
            sleep(1)
            self.explorer.publish('success:1')
        final = RobotModeMsg.MODE_EXPLORATION_RESCUE
        self.agent.to_exploration()
        sleep(10)

        self.assertEqual(self.agent.state_changer.get_current_state(), final)
        self.assertEqual(self.agent.state, 'end')
