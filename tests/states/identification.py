#!/usr/bin/env python

import unittest
from threading import Thread

from mock import patch

import rospy
import roslib
roslib.load_manifest('pandora_fsm')

from rospy import Publisher, sleep
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose
from pandora_data_fusion_msgs.msg import WorldModelMsg

from state_manager_msgs.msg import RobotModeMsg
from pandora_fsm import Agent
from pandora_fsm.mocks import msgs as mock_msgs
import pandora_fsm.config as conf


class TestIdentificationState(unittest.TestCase):
    """ Tests for the identification state. """

    def setUp(self):
        rospy.init_node('state_identification_test')
        self.move_base_mock = Publisher('mock/feedback_move_base', String)
        self.agent = Agent(strategy='normal')
        self.agent.set_breakpoint('victim_deletion')
        self.agent.set_breakpoint('sensor_hold')
        self.agent.target.set(mock_msgs.create_victim_info(id=1,
                                                           probability=0.4))
        self.events = [self.agent.move_base_success,
                       self.agent.move_base_retry,
                       self.agent.move_base_feedback,
                       self.agent.move_base_resend]

    def send_updated_pose(self, delay):
        pose_stamped = PoseStamped()
        pose_stamped.pose.position.x = 20
        pose_stamped.pose.position.y = 20
        target = mock_msgs.create_victim_info(id=1, probability=0.5)
        target.victimPose = pose_stamped
        msg = WorldModelMsg()
        msg.victims = [target]
        msg.visitedVictims = []
        sleep(delay)
        self.agent.receive_world_model(msg)

    def send_identified_victim(self, delay, probability):
        target = mock_msgs.create_victim_info(id=1, probability=probability)
        msg = WorldModelMsg()
        msg.victims = [target]
        msg.visitedVictims = []
        sleep(delay)
        self.agent.receive_world_model(msg)

    def test_global_state_change(self):
        """ The global state should be MODE_IDENTIFICATION. """

        self.move_base_mock.publish('success:10')
        final = RobotModeMsg.MODE_IDENTIFICATION
        self.agent.to_identification()
        sleep(5)
        self.assertEqual(self.agent.state, 'identification')
        self.assertItemsEqual(self.agent.dispatcher.listeners_all(),
                              self.events)
        self.assertEqual(self.agent.state_changer.get_current_state(), final)

    def test_identification_with_move_base(self):
        """ The base has moved to the victim's pose. """

        self.move_base_mock.publish('success:1')
        self.agent.to_identification()
        sleep(5)
        self.assertEqual(self.agent.state, 'sensor_hold')
        self.assertEqual(self.agent.dispatcher.listeners_all(), [])
        self.assertFalse(self.agent.target.is_verified())

    def test_identification_with_high_probability(self):
        """ The move_base has failed but the victim has high probability. """

        self.move_base_mock.publish('abort:2')
        Thread(target=self.send_identified_victim, args=(5, 0.8)).start()
        self.agent.to_identification()
        sleep(15)

        self.assertEqual(self.agent.state, 'sensor_hold')
        self.assertEqual(self.agent.dispatcher.listeners_all(), [])
        self.assertTrue(self.agent.target.is_identified())

    def test_identification_with_convergence(self):
        """ The agent is close enough to the victim to be identified. """

        self.move_base_mock.publish('abort_feedback:2')
        Thread(target=self.send_identified_victim, args=(3, 0.5)).start()
        self.agent.to_identification()
        sleep(20)

        self.assertEqual(self.agent.state, 'sensor_hold')
        self.assertTrue(self.agent.base_converged.is_set())

    def test_abort_victim(self):
        self.move_base_mock.publish('abort:1')
        Thread(target=self.send_identified_victim, args=(3, 0.4)).start()
        self.agent.to_identification()
        sleep(10)

        self.assertEqual(self.agent.state, 'victim_deletion')
        self.assertFalse(self.agent.target.is_identified())

    def test_unresponsive_move_base(self):
        conf.MOVE_BASE_TIMEOUT = 5
        if not rospy.is_shutdown():
            self.move_base_mock.publish('success:7')
        self.agent.to_identification()
        sleep(10)

        self.assertEqual(self.agent.state, 'victim_deletion')

    def test_update_move_base(self):
        if not rospy.is_shutdown():
            self.move_base_mock.publish('execute:10')
        Thread(target=self.send_updated_pose, args=(7, )).start()
        self.agent.to_identification()
        sleep(15)

        x = self.agent.target.info.victimPose.pose.position.x
        y = self.agent.target.info.victimPose.pose.position.y
        self.assertEqual(x, 20)
        self.assertEqual(y, 20)

    def test_update_move_base_timer(self):
        conf.MOVE_BASE_TIMEOUT = 25
        if not rospy.is_shutdown():
            self.move_base_mock.publish('execute:10')
        with patch.object(self.agent.target.dispatcher, 'emit') as mock:
            Thread(target=self.send_updated_pose, args=(5, )).start()
            self.agent.to_identification()
            sleep(10)

            x = self.agent.target.info.victimPose.pose.position.x
            y = self.agent.target.info.victimPose.pose.position.y

            pose = Pose()
            pose.position.x = 20
            pose.position.y = 20
            mock.assert_any_call('move_base.resend', pose)
            self.assertEqual(x, 20)
            self.assertEqual(y, 20)

    def test_no_move_base_update(self):
        self.agent.target.info.victimPose.pose.position.x = 20.1
        self.agent.target.info.victimPose.pose.position.y = 20
        if not rospy.is_shutdown():
            self.move_base_mock.publish('success:10')
        Thread(target=self.send_updated_pose, args=(2, )).start()
        with patch.object(self.agent, 'approach_target') as mock:
            self.agent.to_identification()
            sleep(15)

        x = self.agent.target.info.victimPose.pose.position.x
        y = self.agent.target.info.victimPose.pose.position.y

        # The mock should be called only once because the updated goal is
        # within the acceptable limits
        self.assertEqual(mock.call_count, 1)
        self.assertEqual(x, 20)
        self.assertEqual(y, 20)
