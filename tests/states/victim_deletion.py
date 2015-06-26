#!/usr/bin/env python

import unittest
from time import sleep

import roslib
import rospy

from pandora_fsm import Agent
from pandora_fsm.mocks import msgs as mock_msgs

from rospy import Publisher
from std_msgs.msg import String

roslib.load_manifest('pandora_fsm')


class TestVictimDeletionState(unittest.TestCase):
    """ Tests for the victim deletion state. """

    def setUp(self):
        rospy.init_node('state_victim_deletion_test')
        self.delete_victim_mock = Publisher('mock/delete_victim', String)
        self.agent = Agent(strategy='normal')
        self.agent.set_breakpoint('exploration')

    def test_delete_victim_success(self):
        target = mock_msgs.create_victim_info(id=1, probability=0.7)
        self.agent.available_targets = [target]
        self.agent.target.set(target)

        if not rospy.is_shutdown():
            sleep(1)
            self.delete_victim_mock.publish('success:1')
        self.agent.to_victim_deletion()

        self.assertEqual(self.agent.state, 'exploration')
        self.assertTrue(self.agent.target.is_empty)
        self.assertIn(target, self.agent.deleted_victims)

    def test_delete_victim_fail(self):
        target = mock_msgs.create_victim_info(id=1, probability=0.7)
        self.agent.available_targets = [target]
        self.agent.target.set(target)

        if not rospy.is_shutdown():
            sleep(1)
            self.delete_victim_mock.publish('abort:1')
        self.agent.to_victim_deletion()

        self.assertEqual(self.agent.state, 'exploration')
        self.assertTrue(self.agent.target.is_empty)
        self.assertIn(target, self.agent.deleted_victims)
