#!/usr/bin/env python

"""
    Unit test for the world_model subscriber.
"""

from threading import Thread
import unittest

from mock import MagicMock, patch

import roslib
roslib.load_manifest('pandora_fsm')
import rospy
from rospy import sleep

from pandora_fsm.mocks import msgs as mock_msgs
from pandora_fsm.mocks import WorldModel
from pandora_fsm import Agent, Navigator


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

        self.agent.IDENTIFICATION_THRESHOLD = 0.8
        self.agent.VERIFICATION_THRESHOLD = 0.8

        target = mock_msgs.create_victim_info(id=1)
        target.probability = 0.2

        self.agent.target = target
        visited = [mock_msgs.create_victim_info() for i in range(2)]
        current_victims = [target]
        model = mock_msgs.create_world_model(current_victims, visited)

        with patch.object(Agent, 'update_target_victim') as mock:
            mock.side_effect = empty
            self.agent.receive_world_model(model)

            self.assertTrue(mock.called)
            self.assertFalse(self.agent.probable_victim.is_set())
            self.assertFalse(self.agent.victim_verified.is_set())

    def test_choose_new_victim(self):
        """ Tests that the target is updated. """

        target = mock_msgs.create_victim_info(id=1)
        target.probability = 0.2

        visited = [mock_msgs.create_victim_info() for i in range(2)]
        current_victims = [target]
        model = mock_msgs.create_world_model(current_victims, visited)

        with patch.object(Agent, 'update_target_victim') as mock_update:
            mock_update.side_effect = empty

            self.agent.receive_world_model(model)

            self.assertFalse(mock_update.called)
            self.assertEqual(self.agent.target, target)
            self.assertFalse(self.agent.probable_victim.is_set())
            self.assertFalse(self.agent.victim_verified.is_set())

    def test_poi_found(self):
        """ Tests that the target is updated. """

        self.agent.state = 'exploration'
        target = mock_msgs.create_victim_info(id=1)
        target.probability = 0.2
        self.agent.target = target

        visited = [mock_msgs.create_victim_info() for i in range(2)]
        current_victims = [target]
        model = mock_msgs.create_world_model(current_victims, visited)

        with patch.object(Agent, 'update_target_victim') as mock_update:
            mock_update.side_effect = empty

            self.agent.receive_world_model(model)

            self.assertTrue(mock_update.called)
            self.assertEqual(self.agent.target, target)
            self.assertFalse(self.agent.probable_victim.is_set())
            self.assertFalse(self.agent.victim_verified.is_set())

    def test_identification_threshold(self):
        """ Tests that the probable_victim event is set. """

        self.agent.IDENTIFICATION_THRESHOLD = 0.6
        self.agent.VERIFICATION_THRESHOLD = 0.8

        target = mock_msgs.create_victim_info(probability=0.7)
        self.agent.target = target

        visited = [mock_msgs.create_victim_info() for i in range(2)]
        current_victims = [target]
        model = mock_msgs.create_world_model(current_victims, visited)

        with patch.object(Agent, 'update_target_victim') as mock:
            mock.side_effect = empty
            self.agent.receive_world_model(model)

            self.assertTrue(mock.called)
            self.assertTrue(self.agent.probable_victim.is_set())
            self.assertFalse(self.agent.victim_verified.is_set())

    def test_verification_threshold(self):
        """ Tests that the victim_verified event is set. """

        self.agent.IDENTIFICATION_THRESHOLD = 0.8
        self.agent.VERIFICATION_THRESHOLD = 0.6

        target = mock_msgs.create_victim_info(probability=0.7)
        self.agent.target = target

        visited = [mock_msgs.create_victim_info() for i in range(2)]
        current_victims = [target]
        model = mock_msgs.create_world_model(current_victims, visited)

        with patch.object(Agent, 'update_target_victim') as mock:
            mock.side_effect = empty
            self.agent.receive_world_model(model)

            self.assertTrue(mock.called)
            self.assertFalse(self.agent.probable_victim.is_set())
            self.assertTrue(self.agent.victim_verified.is_set())


class UpdateTarget(unittest.TestCase):

    def setUp(self):
        self.agent = Agent(testing=True)

    def test_with_no_update(self):
        """ The current victim cannot be updated. """

        target = mock_msgs.create_victim_info(id=1, probability=0.6)
        self.agent.target = target
        self.current_victims = []
        approach_target = MagicMock(spec=self.agent.approach_target)

        self.agent.update_target_victim()

        self.assertEqual(self.agent.target.victimPose, target.victimPose)
        self.assertFalse(approach_target.called)

    def test_update_with_valid_target(self):
        """ The target is updated from the victim with the same id. """

        position = mock_msgs.create_point(x=3, y=2, z=1)
        target = mock_msgs.create_victim_info(id=1, probability=0.6)
        target.victimPose.pose.position = position
        position = mock_msgs.create_point(x=1, y=2, z=1)
        target_updated = mock_msgs.create_victim_info(id=1, probability=0.6)
        target_updated.victimPose.pose.position = position

        another = mock_msgs.create_victim_info(id=2, probability=0.3)
        self.agent.target = target
        self.agent.current_victims = [another, target_updated]
        approach_target = MagicMock(spec=self.agent.approach_target)
        cancel_all_goals = MagicMock(spec=self.agent.navigator.cancel_all_goals)

        self.agent.update_target_victim()

        self.assertFalse(approach_target.called)
        self.assertFalse(cancel_all_goals.called)
        self.assertEqual(self.agent.target, target_updated)

    def test_update_with_invalid_target(self):
        """ The target is not updated. """

        position = mock_msgs.create_point(x=3, y=2, z=1)
        target = mock_msgs.create_victim_info(id=1, probability=0.6)
        target.victimPose.pose.position = position
        position = mock_msgs.create_point(x=1, y=2, z=1)
        target_updated = mock_msgs.create_victim_info(id=3, probability=0.6)
        target_updated.victimPose.pose.position = position

        another = mock_msgs.create_victim_info(id=2, probability=0.3)
        self.agent.target = target
        self.agent.current_victims = [another, target_updated]
        approach_target = MagicMock(spec=self.agent.approach_target)
        cancel_all_goals = MagicMock(spec=self.agent.navigator.cancel_all_goals)

        self.agent.update_target_victim()

        self.assertFalse(approach_target.called)
        self.assertFalse(cancel_all_goals.called)
        self.assertEqual(self.agent.target, target)

    def test_new_move_base_goal(self):
        """ Send a new MoveBase goal. """

        self.agent.state = 'identification'
        self.agent.BASE_THRESHOLD = 1

        position = mock_msgs.create_point(x=3, y=2, z=1)
        target = mock_msgs.create_victim_info(id=1, probability=0.6)
        target.victimPose.pose.position = position
        position = mock_msgs.create_point(x=1, y=2, z=1)
        target_updated = mock_msgs.create_victim_info(id=1, probability=0.6)
        target_updated.victimPose.pose.position = position

        another = mock_msgs.create_victim_info(id=2, probability=0.3)
        self.agent.target = target
        self.agent.current_victims = [another, target_updated]

        # Mocks
        with patch.object(Navigator, 'cancel_all_goals') as mock_control:
            with patch.object(Agent, 'approach_target') as mock_agent:

                # Change the implementation of the functions.
                mock_control.side_effect = empty
                mock_agent.side_effect = empty

                self.agent.update_target_victim()

                self.assertTrue(mock_control.called)
                self.assertTrue(mock_agent.called)
                self.assertEqual(self.agent.target, target_updated)
