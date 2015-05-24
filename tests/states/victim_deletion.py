#!/usr/bin/env python

import unittest
from threading import Thread

import rospy
import roslib
roslib.load_manifest('pandora_fsm')

from rospy import Publisher, sleep
from std_msgs.msg import String

from pandora_fsm import Agent
import mock_msgs


class TestVictimDeletionState(unittest.TestCase):
    """ Tests for the victim deletion state. """

    def setUp(self):
        self.delete_victim_mock = Publisher('mock/delete_victim', String)
        self.agent = Agent(strategy='normal')
        self.agent.set_breakpoint('exploration')
        self.world_model = Thread(target=self.send_victim, args=(5,))
        self.target = mock_msgs.create_victim_info(id=1, probability=0.65)
        visited = [mock_msgs.create_victim_info() for i in range(0, 3)]
        self.agent.visited_victims = visited
        self.agent.current_victims.append(self.target)
        self.agent.target = self.target

    def send_victim(self, delay):
        """ Spawn a thread and send a potential victim instead of using a
            mock Publisher which is not so predictable or easy to configure.
        """
        victim = [mock_msgs.create_victim_info(id=1, probability=0.7)]
        visited = [mock_msgs.create_victim_info() for i in range(0, 3)]

        model = mock_msgs.create_world_model(victim, visited)
        sleep(delay)
        self.agent.receive_world_model(model)

    def test_delete_victim_success(self):
        self.delete_victim_mock.publish('success:1')
        self.agent.to_victim_deletion()

        self.assertEqual(self.agent.state, 'exploration')
        self.assertIsNone(self.agent.target)
        self.assertIn(self.target, self.agent.deleted_victims)

    def test_delete_victim_fail(self):
        self.delete_victim_mock.publish('abort:1')
        self.agent.to_victim_deletion()

        self.assertEqual(self.agent.state, 'exploration')
        self.assertIsNone(self.agent.target)
        self.assertIn(self.target, self.agent.deleted_victims)

if __name__ == '__main__':
    rospy.init_node('victim_deletion_state')
    unittest.main()
