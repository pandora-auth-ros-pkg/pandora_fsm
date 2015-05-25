#!/usr/bin/env python

"""
    Unit test for the world_model subscriber.
"""

import unittest

import roslib
roslib.load_manifest('pandora_fsm')
import rospy
from rospy import Publisher, sleep
from std_msgs.msg import String

from pandora_fsm.mocks import msgs as mock_msgs
from pandora_fsm import Agent


class TestWorldModelSubscriber(unittest.TestCase):
    """ Tests for the agent callbacks. """

    def setUp(self):
        self.agent = Agent(strategy='normal')
        self.world_model = Publisher('mock/world_model', String)

    def test_receive_world_model_response(self):
        # To assert point_of_interest
        self.agent.state = 'exploration'
        while not rospy.is_shutdown():
            self.world_model.publish('1')
            break
        sleep(3)
        self.assertNotEqual(self.agent.current_victims, [])
        self.assertNotEqual(self.agent.visited_victims, [])
        self.assertTrue(self.agent.point_of_interest.is_set())

    def test_receive_world_model_with_target(self):
        """ Tests that the target is updated. """

        # To assert point_of_interest
        self.agent.state = 'exploration'

        # Create a victim and assign it to the target.
        target = mock_msgs.create_victim_info()
        self.agent.target_victim = target

        self.assertEqual(self.agent.target_victim.id, target.id)

        # Create a custom world_model msg with updated target.
        target.probability = 0.8
        visited = [mock_msgs.create_victim_info() for i in range(2)]
        current_victims = [target]
        model = mock_msgs.create_world_model(current_victims, visited)

        self.agent.receive_world_model(model)
        self.assertEqual(self.agent.target_victim.id, target.id)
        self.assertEqual(self.agent.target_victim.probability, 0.8)
        self.assertTrue(self.agent.point_of_interest.is_set())

    def test_receive_world_model_identification_threshold(self):
        """ Tests that the promising_victim event is set. """

        # To assert point_of_interest
        self.agent.state = 'exploration'
        self.agent.IDENTIFICATION_THRESHOLD = 0.5
        target = mock_msgs.create_victim_info(probability=0.7)
        self.agent.target_victim = target

        visited = [mock_msgs.create_victim_info() for i in range(2)]
        current_victims = [target]
        model = mock_msgs.create_world_model(current_victims, visited)

        self.agent.receive_world_model(model)
        self.assertEqual(self.agent.target_victim.id, target.id)
        self.assertTrue(self.agent.promising_victim.is_set())

        target.probability = 0.2
        current_victims = [target]
        model = mock_msgs.create_world_model(current_victims, visited)

        self.agent.receive_world_model(model)
        self.assertEqual(self.agent.target_victim.id, target.id)
        self.assertFalse(self.agent.promising_victim.is_set())

    def test_receive_world_model_verification_threshold(self):
        """ Tests that the recognized_victim event is set. """

        # To assert point_of_interest
        self.agent.state = 'exploration'
        self.agent.VERIFICATION_THRESHOLD = 0.5
        target = mock_msgs.create_victim_info(probability=0.7)
        self.agent.target_victim = target

        visited = [mock_msgs.create_victim_info() for i in range(2)]
        current_victims = [target]
        model = mock_msgs.create_world_model(current_victims, visited)

        self.agent.receive_world_model(model)
        self.assertEqual(self.agent.target_victim.id, target.id)
        self.assertTrue(self.agent.recognized_victim.is_set())

        target.probability = 0.2
        current_victims = [target]
        model = mock_msgs.create_world_model(current_victims, visited)

        self.agent.receive_world_model(model)
        self.assertEqual(self.agent.target_victim.id, target.id)
        self.assertFalse(self.agent.recognized_victim.is_set())


if __name__ == '__main__':
    rospy.init_node('unit_world_model')
    unittest.main()
