#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2014, P.A.N.D.O.R.A. Team.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of P.A.N.D.O.R.A. Team nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Voulgarakis George <turbolapingr@gmail.com>

import roslib
roslib.load_manifest('pandora_fsm')
import rospy
import unittest
import global_vars

from threading import Thread
from pandora_fsm.agent_states import data_fusion_hold_state, \
    exploration_strategy4_state, identification_check_for_victims_state, \
    teleoperation_state

from state_manager_communications.msg import robotModeMsg
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from pandora_data_fusion_msgs.msg import WorldModelMsg, VictimInfoMsg, \
    QrNotificationMsg
from pandora_navigation_msgs.msg import ArenaTypeMsg, DoExplorationGoal


class TestAgent(unittest.TestCase):

    def test_robot_start_state(self):
        rospy.sleep(2.)
        rospy.loginfo('Simulate start button')
        global_vars.test_agent.transition_to_state(robotModeMsg.
                                                   MODE_START_AUTONOMOUS)
        rospy.sleep(22.)
        self.assertIsInstance(global_vars.test_agent.current_state_,
                              exploration_strategy4_state.
                              ExplorationStrategy4State)
        self.assertEqual(global_vars.test_agent.current_robot_state_,
                         robotModeMsg.MODE_EXPLORATION)

    def test_exploration_state(self):
        rospy.sleep(10.)
        rospy.loginfo('Publish a victim')
        victims_to_go = WorldModelMsg()
        victim = VictimInfoMsg()
        victim.id = 5
        victim.victimPose.pose.position.x = \
            global_vars.com.robot_pose_.pose.position.x - 1.1
        victim.victimPose.pose.position.y = \
            global_vars.com.robot_pose_.pose.position.y - 0.5
        victim.victimPose.pose.position.z = 0.4
        victim.victimPose.pose.orientation.x = 0.1
        victim.victimPose.pose.orientation.y = 0.5
        victim.victimPose.pose.orientation.z = 1
        victim.victimPose.pose.orientation.w = 0.9
        victim.victimPose.header.frame_id = "dummy_frame"
        victim.victimFrameId = "dummy_frame"
        victim.probability = 0.0
        victims_to_go.victims.append(victim)
        global_vars.com.victims_pub_.publish(victims_to_go)

        rospy.sleep(3.)
        self.assertIsInstance(global_vars.test_agent.current_state_,
                              identification_check_for_victims_state.
                              IdentificationCheckForVictimsState)
        self.assertEqual(global_vars.test_agent.current_robot_state_,
                         robotModeMsg.MODE_IDENTIFICATION)
        self.assertEqual(global_vars.test_agent.current_exploration_mode_, -1)

    def test_identification_state(self):
        rospy.loginfo('Update target victim')
        victims_to_go = WorldModelMsg()
        victim = VictimInfoMsg()
        victim.id = 5
        victim.victimPose.pose.position.x = \
            global_vars.test_agent.\
            target_victim_.victimPose.pose.position.x - 0.1
        victim.victimPose.pose.position.y = \
            global_vars.test_agent.target_victim_.victimPose.pose.position.y
        victim.victimPose.pose.position.z = 0.4
        victim.victimPose.pose.orientation.x = 0.1
        victim.victimPose.pose.orientation.y = 0.5
        victim.victimPose.pose.orientation.z = 1
        victim.victimPose.pose.orientation.w = 0.9
        victim.victimPose.header.frame_id = "dummy_frame"
        victim.victimFrameId = "dummy_frame"
        victim.probability = 0.0
        victims_to_go.victims.append(victim)
        global_vars.com.victims_pub_.publish(victims_to_go)
        rospy.sleep(16.)
        self.assertIsInstance(global_vars.test_agent.current_state_,
                              data_fusion_hold_state.DataFusionHoldState)
        self.assertEqual(global_vars.test_agent.current_robot_state_,
                         robotModeMsg.MODE_DF_HOLD)

    def test_validation_state(self):
        rospy.loginfo('Publish target victim with increased probability')
        victims_to_go = WorldModelMsg()
        victim = VictimInfoMsg()
        victim.id = 5
        victim.victimPose.pose.position.x = \
            global_vars.test_agent.target_victim_.victimPose.pose.position.x
        victim.victimPose.pose.position.y = \
            global_vars.test_agent.target_victim_.victimPose.pose.position.y
        victim.victimPose.pose.position.z = 0.4
        victim.victimPose.pose.orientation.x = 0.1
        victim.victimPose.pose.orientation.y = 0.5
        victim.victimPose.pose.orientation.z = 1
        victim.victimPose.pose.orientation.w = 0.9
        victim.victimPose.header.frame_id = "dummy_frame"
        victim.victimFrameId = "dummy_frame"
        victim.probability = 0.8
        victim.sensors.append('FACE')
        victim.sensors.append('THERMAL')
        victims_to_go.victims.append(victim)
        global_vars.com.victims_pub_.publish(victims_to_go)
        rospy.sleep(19.)
        self.assertEqual(global_vars.test_agent.valid_victims_, 1)
        self.assertIsInstance(global_vars.test_agent.current_state_,
                              exploration_strategy4_state.
                              ExplorationStrategy4State)
        self.assertEqual(global_vars.test_agent.current_robot_state_,
                         robotModeMsg.MODE_EXPLORATION)

    def test_exploration_state_change_to_normal(self):
        rospy.loginfo('Change exploration type to normal')
        rospy.sleep(1.)
        msg = QrNotificationMsg()
        for i in range(10):
            global_vars.com.qr_notification_pub_.publish(msg)
            rospy.Rate(10).sleep()
        global_vars.test_agent.initial_time_ = rospy.get_rostime().secs - 400
        msg = Float32(data=10)
        global_vars.com.area_covered_pub_.publish(msg)
        rospy.sleep(2.)
        self.assertEqual(global_vars.test_agent.current_exploration_mode_,
                         DoExplorationGoal.TYPE_NORMAL)

    def test_teleoperation_state(self):
        rospy.loginfo('Go to teleoperation')
        global_vars.test_agent.transition_to_state(robotModeMsg.
                                                   MODE_TELEOPERATED_LOCOMOTION)
        rospy.sleep(1.)
        global_vars.com.move_end_effector_succeeded_ = True
        rospy.sleep(2.)
        self.assertIsInstance(global_vars.test_agent.current_state_,
                              teleoperation_state.TeleoperationState)
        self.assertEqual(global_vars.test_agent.current_robot_state_,
                         robotModeMsg.MODE_TELEOPERATED_LOCOMOTION)

if __name__ == '__main__':
    rospy.init_node('test_agent')
    global_vars.init()
    rospy.sleep(1.)

    #start the agent in a new thread
    agent_thread = Thread(target=global_vars.test_agent.main)
    agent_thread.start()

    #test ROBOT_START state
    robot_start_state = unittest.TestSuite()
    robot_start_state.addTest(TestAgent('test_robot_start_state'))
    unittest.TextTestRunner(verbosity=1).run(robot_start_state)

    #test EXPLORATION state
    exploration_state = unittest.TestSuite()
    exploration_state.addTest(TestAgent('test_exploration_state'))
    unittest.TextTestRunner(verbosity=1).run(exploration_state)

    #test IDENTIFICATION state
    #update the victim and then move to it
    victim_monitoring_state = unittest.TestSuite()
    victim_monitoring_state.addTest(TestAgent('test_identification_state'))
    unittest.TextTestRunner(verbosity=1).run(victim_monitoring_state)

    #test VALIDATION state
    victim_validation_state = unittest.TestSuite()
    victim_validation_state.addTest(TestAgent('test_validation_state'))
    unittest.TextTestRunner(verbosity=1).run(victim_validation_state)

    #change EXPLORATION type to TYPE_NORMAL
    exploration_normal = unittest.TestSuite()
    exploration_normal.\
        addTest(TestAgent('test_exploration_state_change_to_normal'))
    unittest.TextTestRunner(verbosity=1).run(exploration_normal)

    #test TELEOPERATION state
    teleoperation = unittest.TestSuite()
    teleoperation.addTest(TestAgent('test_teleoperation_state'))
    unittest.TextTestRunner(verbosity=1).run(teleoperation)

    global_vars.com.delete_action_servers()
    rospy.signal_shutdown('Functional tests finished')
