#!/usr/bin/env python

import unittest
from threading import Thread

from mock import patch

import rospy
import roslib
roslib.load_manifest('pandora_fsm')

from rospy import Publisher, sleep
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from pandora_data_fusion_msgs.msg import WorldModelMsg

from state_manager_msgs.msg import RobotModeMsg
from pandora_fsm import Agent
from pandora_fsm.mocks import msgs as mock_msgs


class TestIdentificationState(unittest.TestCase):
    """ Tests for the identification state. """

    def setUp(self):
        self.effector_mock = Publisher('mock/effector', String)
        self.move_base_mock = Publisher('mock/feedback_move_base', String)
        self.victim_mock = Publisher('mock/victim_probability', String)
        self.agent = Agent(strategy='normal')
        self.agent.set_breakpoint('victim_deletion')
        self.agent.set_breakpoint('sensor_hold')
        target = mock_msgs.create_victim_info(id=8, probability=0.4)
        self.world_model = Thread(target=self.send_updated_pose, args=(3,))
        self.agent.target = target

    def send_updated_pose(self, delay):
        pose_stamped = PoseStamped()
        pose_stamped.pose.position.x = 20
        pose_stamped.pose.position.y = 20
        target = mock_msgs.create_victim_info(id=8, probability=0.4)
        target.victimPose = pose_stamped
        msg = WorldModelMsg()
        msg.victims = [target]
        msg.visitedVictims = []
        sleep(delay)
        self.agent.receive_world_model(msg)

    def test_global_state_change(self):
        """ The global state should be MODE_IDENTIFICATION. """

        self.move_base_mock.publish('success:2')
        if not rospy.is_shutdown():
            self.victim_mock.publish('5:0.6')
        final = RobotModeMsg.MODE_IDENTIFICATION
        self.agent.to_identification()
        sleep(5)

        self.assertEqual(self.agent.state, 'sensor_hold')
        self.assertEqual(self.agent.state_changer.get_current_state(), final)

    def test_found_victim(self):
        """ The base has moved to the victim's pose. """

        self.move_base_mock.publish('success:1')
        if not rospy.is_shutdown():
            self.victim_mock.publish('3:0.6')
        self.agent.to_identification()
        sleep(5)
        self.assertEqual(self.agent.state, 'sensor_hold')

    def test_abort_with_high_probability(self):
        """ The move_base has failed but the victim has high probability. """

        self.agent.IDENTIFICATION_THRESHOLD = 0.6
        self.move_base_mock.publish('abort:2')
        if not rospy.is_shutdown():
            self.victim_mock.publish('8:0.9')
        self.agent.to_identification()
        sleep(15)

        self.assertEqual(self.agent.state, 'sensor_hold')
        self.assertTrue(self.agent.probable_victim.is_set())

    def test_abort_with_convergence(self):
        """ The agent is close enough to the victim to be identified. """

        self.move_base_mock.publish('abort_feedback:2')
        self.agent.IDENTIFICATION_THRESHOLD = 0.7
        if not rospy.is_shutdown():
            self.victim_mock.publish('5:0.6')
        self.agent.to_identification()
        sleep(20)

        self.assertEqual(self.agent.state, 'sensor_hold')
        self.assertTrue(self.agent.base_converged.is_set())

    def test_abort_victim(self):
        self.move_base_mock.publish('abort:1')
        if not rospy.is_shutdown():
            self.victim_mock.publish('8:0.5')
        self.agent.to_identification()
        sleep(10)

        self.assertEqual(self.agent.state, 'victim_deletion')

    def test_unresponsive_move_base(self):
        self.agent.MOVE_BASE_TIMEOUT = 5
        if not rospy.is_shutdown():
            self.move_base_mock.publish('success:7')
        self.agent.to_identification()
        sleep(10)

        self.agent.to_identification()
        sleep(10)

        self.assertEqual(self.agent.state, 'victim_deletion')

    def test_update_move_base(self):
        if not rospy.is_shutdown():
            self.move_base_mock.publish('execute:10')
        self.world_model.start()
        self.agent.to_identification()
        sleep(15)

        x = self.agent.target.victimPose.pose.position.x
        y = self.agent.target.victimPose.pose.position.y
        self.assertEqual(x, 20)
        self.assertEqual(y, 20)

    def test_update_move_base_timer(self):
        self.agent.MOVE_BASE_TIMEOUT = 5
        if not rospy.is_shutdown():
            self.move_base_mock.publish('execute:10')
        self.world_model.start()
        with patch.object(self.agent, 'approach_target') as mock:
            self.agent.to_identification()
            sleep(15)

        x = self.agent.target.victimPose.pose.position.x
        y = self.agent.target.victimPose.pose.position.y

        # The mock should be called another time after the update of the
        # move base goal.
        self.assertEqual(mock.call_count, 2)
        self.assertEqual(x, 20)
        self.assertEqual(y, 20)

    def test_no_move_base_update(self):
        self.agent.target.victimPose.pose.position.x = 20.1
        self.agent.target.victimPose.pose.position.y = 20
        if not rospy.is_shutdown():
            self.move_base_mock.publish('success:10')
        self.world_model.start()
        with patch.object(self.agent, 'approach_target') as mock:
            self.agent.to_identification()
            sleep(15)

        x = self.agent.target.victimPose.pose.position.x
        y = self.agent.target.victimPose.pose.position.y

        # The mock should be called only once because the updated goal is
        # within the acceptable limits
        self.assertEqual(mock.call_count, 1)
        self.assertEqual(x, 20)
        self.assertEqual(y, 20)


if __name__ == '__main__':
    rospy.init_node('identification_state')
    unittest.main()
