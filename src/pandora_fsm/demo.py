"""
    Testing the FSM framework with ROS.

    The test simulates the `wait for external event` task, using:
    - publisher/subscriber
    - services
    - actions
"""

PKG = 'pandora_fsm'

import unittest

import rostest
import rospy

from agent import Agent


class TestFiniteStateMahine(unittest.TestCase):

    def setUp(self):
        self.woody = Agent()

    def test_initialization(self):
        self.assertEqual(self.woody.name, 'Pandora')
        self.assertEqual(self.woody.state, 'off')

    #def test_wait_for_publisher(self):

        ## We go in exploration waiting for victims.
        #self.woody.to_exploration()

        ## We found one.
        #self.assertTrue(self.woody.victim)

        ## Moving to the next state.
        #self.assertEqual(self.woody.state, 'identification')

    def test_wait_for_action(self):

        # We go close to the victim.
        self.woody.to_closeup()

        # Wait for validation.
        if self.woody.gui_verification == True:
            self.assertEqual(self.woody.state, 'operator_validation')
        elif self.woody.is_timeout == True:
            self.assertEqual(self.woody.state, 'fusion_validation')
        else:
            # Fail
            self.assertEqual(True, False)


if __name__ == '__main__':
    rostest.rosrun(PKG, 'test_fsm', TestFiniteStateMahine)
