#!/usr/bin/env python

"""
    Unit test for the world_model subscriber.
"""

from threading import Thread
import unittest

from mock import patch

import roslib
roslib.load_manifest('pandora_fsm')
from time import sleep

from pandora_fsm.mocks import msgs as mock_msgs
from pandora_fsm.mocks import WorldModel
import pandora_fsm.config as conf
from pandora_fsm import Agent


def empty():
    """ Dummy patch function. """
    pass


class WorldModelSub(unittest.TestCase):
    """ Tests for the agent callbacks. """

    def setUp(self):
        self.agent = Agent(testing=True)
        self.world_model = WorldModel()

    @unittest.skip('ROS dependent')
    def test_connection(self):
        """ The receive_world_model should be called. """

        # Set up testing mocks
        self.agent.target = None
        self.agent.state = 'exploration'
        Thread(target=self.world_model.send_random).start()
        sleep(3)

        self.assertIsNot(self.agent.current_victims, [])
        self.assertIsNot(self.agent.visited_victims, [])
        self.assertIsNotNone(self.agent.target)

    def test_update_target(self):
        """ Tests that the target is updated. """

        target = mock_msgs.create_victim_info(id=1, probability=0.2)

        self.agent.target.set(target)
        visited = [mock_msgs.create_victim_info() for i in range(2)]
        current_victims = [target]
        model = mock_msgs.create_world_model(current_victims, visited)

        with patch.object(self.agent.target, 'update') as mock:
            with patch.object(self.agent.dispatcher, 'emit') as emitter:
                self.agent.receive_world_model(model)

                self.assertTrue(mock.called)
                self.assertTrue(emitter.called)
                self.assertFalse(self.agent.target.is_verified())
                self.assertFalse(self.agent.target.is_identified())

    def test_choose_new_victim(self):
        """ Tests that the target is updated. """

        target = mock_msgs.create_victim_info(id=1, probability=0.2)

        visited = [mock_msgs.create_victim_info() for i in range(2)]
        current_victims = [target]
        model = mock_msgs.create_world_model(current_victims, visited)

        with patch.object(self.agent.target, 'update') as mock_update:
            with patch.object(self.agent.dispatcher, 'emit') as emitter:
                self.agent.receive_world_model(model)

                self.assertFalse(mock_update.called)
                self.assertTrue(emitter.called)
                self.assertEqual(self.agent.target.info, target)
                self.assertFalse(self.agent.target.is_verified())
                self.assertFalse(self.agent.target.is_identified())

    def test_non_existent_victim(self):
        target = mock_msgs.create_victim_info(id=1, probability=0.2)
        target2 = mock_msgs.create_victim_info(id=2, probability=0.2)

        visited = [mock_msgs.create_victim_info() for i in range(2)]
        current_victims = [target2]
        model = mock_msgs.create_world_model(current_victims, visited)
        self.agent.target.set(target)

        with patch.object(self.agent.target, 'update') as mock_update:
            with patch.object(self.agent.dispatcher, 'emit') as emitter:
                self.agent.receive_world_model(model)

                self.assertFalse(mock_update.called)
                self.assertFalse(emitter.called)
                self.assertTrue(self.agent.target.is_empty)
                self.assertFalse(self.agent.target.is_verified())
                self.assertFalse(self.agent.target.is_identified())

    def test_identification_threshold(self):
        """ Tests that the target is identified. """

        conf.IDENTIFICATION_THRESHOLD = 0.6
        conf.VERIFICATION_THRESHOLD = 0.8

        target = mock_msgs.create_victim_info(probability=0.7)
        self.agent.target.set(target)

        visited = [mock_msgs.create_victim_info() for i in range(2)]
        current_victims = [target]
        model = mock_msgs.create_world_model(current_victims, visited)

        self.agent.receive_world_model(model)

        self.assertFalse(self.agent.target.is_empty)
        self.assertTrue(self.agent.target.is_identified())
        self.assertFalse(self.agent.target.is_verified())

    def test_verification_threshold(self):
        """ Tests that the target is verified. """

        conf.IDENTIFICATION_THRESHOLD = 0.6
        conf.VERIFICATION_THRESHOLD = 0.8

        target = mock_msgs.create_victim_info(probability=0.9)
        self.agent.target.set(target)

        visited = [mock_msgs.create_victim_info() for i in range(2)]
        current_victims = [target]
        model = mock_msgs.create_world_model(current_victims, visited)

        self.agent.receive_world_model(model)

        self.assertFalse(self.agent.target.is_empty)
        self.assertTrue(self.agent.target.is_identified())
        self.assertTrue(self.agent.target.is_verified())
