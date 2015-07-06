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
from pandora_fsm.mocks import WorldModelPub
import pandora_fsm.config as conf
from pandora_fsm import Agent


def empty():
    """ Dummy patch function. """
    pass


class WorldModelSub(unittest.TestCase):
    """ Tests for the agent callbacks. """

    def setUp(self):
        self.agent = Agent(testing=True)
        self.world_model = WorldModelPub()

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


class ChooseTarget(unittest.TestCase):

    """ Test choose_target function. """

    def setUp(self):
        self.agent = Agent(testing=True)
        self.agent.current_pose = mock_msgs.create_pose(x=0, y=0)

    def test_empty_targets(self):
        """ Should throw an error and return None. """

        targets = []
        next_target = self.agent.choose_target(targets)

        self.assertIsNone(next_target)

    def test_with_one_target(self):
        """ Should return the one target. """

        target = mock_msgs.create_victim_info()
        next_target = self.agent.choose_target([target])

        self.assertEqual(target, next_target)

    def test_equal_distance_targets(self):
        """ Should return the first target. """

        target1 = mock_msgs.create_victim_info(id=1)
        target1.victimPose = mock_msgs.create_pose_stamped(x=1, y=1)
        target2 = mock_msgs.create_victim_info(id=2)
        target2.victimPose = mock_msgs.create_pose_stamped(x=1, y=1)

        next_target = self.agent.choose_target([target1, target2])
        self.assertEqual(next_target, target1)

    def test_with_normal_targets(self):
        """ Should return the nearest target. """

        target1 = mock_msgs.create_victim_info(id=1)
        target1.victimPose = mock_msgs.create_pose_stamped(x=1, y=1)
        target2 = mock_msgs.create_victim_info(id=2)
        target2.victimPose = mock_msgs.create_pose_stamped(x=2, y=2)
        target3 = mock_msgs.create_victim_info(id=3)
        target3.victimPose = mock_msgs.create_pose_stamped(x=3, y=3)

        next_target = self.agent.choose_target([target1, target2, target3])
        self.assertEqual(next_target, target1)
