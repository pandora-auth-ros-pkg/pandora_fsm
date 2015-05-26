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

from pandora_fsm.mocks import msgs as mock_msgs


class TestExplorationState(unittest.TestCase):
    """ Tests for the exploration state. """

    def setUp(self):
        self.agent = Agent(strategy='normal')
        self.agent.set_breakpoint('identification')
        self.agent.set_breakpoint('end')
        self.agent.set_breakpoint('init')
        self.effector_mock = Publisher('mock/effector', String)
        self.explorer = Publisher('mock/explorer', String)
        self.world_model = Thread(target=self.send_victim, args=(3,))

    def send_victim(self, delay):
        """ Spawn a thread and send a potential victim instead of using a
            mock Publisher which is not so predictable or easy to configure.
        """
        visited = [mock_msgs.create_victim_info() for i in range(0, 3)]
        victim = [mock_msgs.create_victim_info()]

        model = mock_msgs.create_world_model(victim, visited)
        sleep(delay)
        self.agent.receive_world_model(model)

    def test_to_identification(self):

        # Long goals that will not affect the test.
        if not rospy.is_shutdown():
            sleep(1)
            self.effector_mock.publish('success:20')
            self.explorer.publish('success:20')
        self.world_model.start()
        self.agent.to_exploration()
        sleep(10)
        self.assertEqual(self.agent.state, 'identification')
        self.assertFalse(self.agent.state_can_change.is_set())

    def test_race_condition(self):
        if not rospy.is_shutdown():
            sleep(1)
            self.explorer.publish('success:1')
            self.world_model.start()
        self.agent.to_exploration()
        sleep(10)
        self.assertFalse(self.agent.state_can_change.is_set())
        self.assertEqual(self.agent.state, 'end')

    def test_to_end(self):
        self.effector_mock.publish('success:10')

        # This goal will move the agent to the end state.
        if not rospy.is_shutdown():
            sleep(1)
            self.explorer.publish('success:1')

        self.agent.to_exploration()
        sleep(15)
        self.assertEqual(self.agent.state, 'end')
        self.assertFalse(self.agent.state_can_change.is_set())

    def test_long_wait_for_victim(self):

        # Long goals that will not affect the test.
        if not rospy.is_shutdown():
            sleep(1)
            self.effector_mock.publish('success:20')
            self.explorer.publish('success:40')

        self.agent.to_exploration()
        sleep(20)
        self.assertEqual(self.agent.state, 'exploration')
        self.assertTrue(self.agent.state_can_change.is_set())

    def test_retry_on_explorer_abort(self):
        """ The agent will keep sending goals if the explorer fails. """

        if not rospy.is_shutdown():
            sleep(1)
            self.explorer.publish('abort:1')
            self.explorer.publish('abort:1')

        with patch.object(self.agent.explorer.dispatcher, 'emit') as mock:
            self.agent.to_exploration()
            sleep(7)
        mock.assert_called_with('exploration.retry')
        self.assertEqual(self.agent.state, 'exploration')
        self.assertTrue(self.agent.state_can_change.is_set())

    def test_retry_on_explorer_reject(self):
        """ The agent will keep sending goals if the explorer fails. """

        if not rospy.is_shutdown():
            sleep(1)
            self.explorer.publish('reject:1')
            self.explorer.publish('reject:1')
        with patch.object(self.agent.explorer.dispatcher, 'emit') as mock:
            self.agent.to_exploration()
            sleep(7)
        mock.assert_called_with('exploration.retry')
        self.assertEqual(self.agent.state, 'exploration')
        self.assertTrue(self.agent.state_can_change.is_set())

    def test_global_state_change(self):
        """ The global state should be MODE_EXPLORATION_RESCUE """

        if not rospy.is_shutdown():
            sleep(1)
            self.effector_mock.publish('success:20')
            self.explorer.publish('success:1')
        final = RobotModeMsg.MODE_EXPLORATION_RESCUE
        self.agent.to_exploration()
        sleep(10)

        self.assertEqual(self.agent.state_changer.get_current_state(), final)
        self.assertEqual(self.agent.state, 'end')
        self.assertFalse(self.agent.state_can_change.is_set())


if __name__ == '__main__':
    rospy.init_node('exploration_state')
    unittest.main()
