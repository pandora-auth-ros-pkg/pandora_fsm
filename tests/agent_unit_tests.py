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

from state_manager_communications.msg import robotModeMsg
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Int32
from pandora_fsm.robocup_agent.robocup_cost_functions import \
    ExplorationModeCostFunction, ExplorationModeCostFunction2, \
    FindNewVictimToGoCostFunction, UpdateVictimCostFunction, ExplorationModeCostFunction3
from pandora_data_fusion_msgs.msg import VictimInfoMsg, QrNotificationMsg
from pandora_navigation_msgs.msg import ArenaTypeMsg, DoExplorationGoal


class TestAgent(unittest.TestCase):

    def tearDown(self):
        global_vars.com.test_passed_ = False
        global_vars.com.robot_pose_ = PoseStamped()
        global_vars.test_agent.current_arena_ = ArenaTypeMsg.TYPE_YELLOW
        global_vars.test_agent.area_explored_ = 0
        global_vars.test_agent.current_score_ = 0
        global_vars.test_agent.valid_victims_ = 0
        global_vars.test_agent.qrs_ = 0
        global_vars.test_agent.current_robot_pose_ = PoseStamped()
        global_vars.test_agent.current_exploration_mode_ = -1
        global_vars.test_agent.aborted_victims_ = []
        global_vars.test_agent.new_victims_ = []
        global_vars.test_agent.target_victim_ = VictimInfoMsg()

        global_vars.test_agent.robot_resets_ = 0
        global_vars.test_agent.robot_restarts_ = 0

        global_vars.test_agent.new_robot_state_ = robotModeMsg.MODE_OFF
        global_vars.test_agent.new_robot_state_ack_ = robotModeMsg.MODE_OFF
        global_vars.test_agent.current_robot_state_ = robotModeMsg.MODE_OFF

        global_vars.test_agent.previous_state_ = "waiting_to_start_state"

        global_vars.test_agent.current_state_ = \
            global_vars.test_agent.all_states_["waiting_to_start_state"]

        global_vars.com.dynamic_reconfigure_client.\
            update_configuration({"arenaVictims": 4, "maxQRs": 20,
                                  "arenaArea": 18, "maxTime": 1200,
                                  "timePassed": 0,
                                  "explorationStrategy": 2,
                                  "validVictimProbability": 0.5,
                                  "abortedVictimsDistance": 0.05,
                                  "deepLimit": 0.85})

    def test_arena_type(self):
        rospy.loginfo('test_arena_type')
        msg = ArenaTypeMsg(arena_type=ArenaTypeMsg.TYPE_ORANGE)
        global_vars.com.arena_type_pub_.publish(msg)
        rospy.Rate(10).sleep()
        self.assertEqual(global_vars.test_agent.
                         current_arena_, ArenaTypeMsg.TYPE_ORANGE)

    def test_calculate_distance_2d(self):
        rospy.loginfo('test_calculate_distance_2d')
        object1 = Point()
        object1.x = 2
        object1.y = 4
        object2 = Point()
        object2.x = 7
        object2.y = 3
        dist = global_vars.test_agent.calculate_distance_2d(object1, object2)
        self.assertEqual(dist, 5.0990195135927845)

    def test_calculate_distance_3d(self):
        rospy.loginfo('test_calculate_distance_3d')
        object1 = Point()
        object1.x = 2
        object1.y = 4
        object1.z = 5
        object2 = Point()
        object2.x = 7
        object2.y = 3
        object2.z = 1
        dist = global_vars.test_agent.calculate_distance_3d(object1, object2)
        self.assertEqual(dist, 6.48074069840786)

    def test_data_fusion_hold_state_exploration(self):
        rospy.loginfo('test_data_fusion_hold_state_exploration')
        global_vars.test_agent.new_robot_state_ = robotModeMsg.MODE_DF_HOLD
        global_vars.test_agent.\
            all_states_["data_fusion_hold_state"].counter_ = 10
        global_vars.test_agent.current_robot_pose_.pose.position.x = 4.8
        global_vars.test_agent.current_robot_pose_.pose.position.y = 11.6
        victims = []
        victim = VictimInfoMsg()
        victim.id = 8
        victim.victimPose.pose.position.x = 2
        victim.victimPose.pose.position.y = 6
        victim.victimPose.pose.position.z = 3.5
        victim.probability = 0.3
        victims.append(victim)
        global_vars.test_agent.new_victims_ = victims
        victim = VictimInfoMsg()
        victim.id = 8
        victim.victimPose.pose.position.x = 2
        victim.victimPose.pose.position.y = 6
        victim.victimPose.pose.position.z = 3.5
        victim.probability = 0.2
        global_vars.test_agent.target_victim_ = victim
        next_state = global_vars.test_agent.\
            all_states_["data_fusion_hold_state"].make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "exploration_strategy2_state")
        self.assertEqual(global_vars.test_agent.current_robot_state_,
                         robotModeMsg.MODE_EXPLORATION)

    def test_data_fusion_hold_state_find_new_victim(self):
        rospy.loginfo('test_data_fusion_hold_state_find_new_victim')
        global_vars.test_agent.new_robot_state_ = robotModeMsg.MODE_DF_HOLD
        global_vars.test_agent.\
            all_states_["data_fusion_hold_state"].counter_ = 10
        global_vars.test_agent.current_robot_pose_.pose.position.x = 1.8
        global_vars.test_agent.current_robot_pose_.pose.position.y = 8.6
        victims = []
        victim = VictimInfoMsg()
        victim.id = 8
        victim.victimPose.pose.position.x = 2
        victim.victimPose.pose.position.y = 6
        victim.victimPose.pose.position.z = 3.5
        victim.probability = 0.3
        victims.append(victim)
        victim = VictimInfoMsg()
        victim.id = 9
        victim.victimPose.pose.position.x = 2
        victim.victimPose.pose.position.y = 9
        victim.victimPose.pose.position.z = 3.2
        victim.probability = 0.2
        victims.append(victim)
        global_vars.test_agent.new_victims_ = victims
        victim = VictimInfoMsg()
        victim.id = 8
        victim.victimPose.pose.position.x = 2
        victim.victimPose.pose.position.y = 6
        victim.victimPose.pose.position.z = 3.5
        victim.probability = 0.2
        global_vars.test_agent.target_victim_ = victim
        next_state = global_vars.test_agent.\
            all_states_["data_fusion_hold_state"].make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "identification_state")
        self.assertEqual(global_vars.test_agent.current_robot_state_,
                         robotModeMsg.MODE_IDENTIFICATION)

    def test_data_fusion_hold_state_teleoperation(self):
        rospy.loginfo('test_data_fusion_hold_state_teleoperation')
        global_vars.test_agent.new_robot_state_ = robotModeMsg.MODE_DF_HOLD
        global_vars.test_agent.transition_to_state(robotModeMsg.
                                                   MODE_TELEOPERATED_LOCOMOTION)
        rospy.Rate(10).sleep()
        next_state = global_vars.test_agent.\
            all_states_["data_fusion_hold_state"].make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "teleoperation_state")
        self.assertEqual(global_vars.test_agent.current_robot_state_,
                         robotModeMsg.MODE_TELEOPERATED_LOCOMOTION)

    def test_data_fusion_hold_state_validation(self):
        rospy.loginfo('test_data_fusion_hold_state_validation')
        global_vars.test_agent.new_robot_state_ = robotModeMsg.MODE_DF_HOLD
        global_vars.test_agent.\
            all_states_["data_fusion_hold_state"].counter_ = 10
        victims = []
        victim = VictimInfoMsg()
        victim.id = 8
        victim.victimPose.pose.position.x = 2
        victim.victimPose.pose.position.y = 6
        victim.victimPose.pose.position.z = 3.2
        victim.probability = 0.9
        victims.append(victim)
        global_vars.test_agent.new_victims_ = victims
        victim = VictimInfoMsg()
        victim.id = 8
        victim.victimPose.pose.position.x = 2
        victim.victimPose.pose.position.y = 6
        victim.victimPose.pose.position.z = 3.2
        victim.probability = 0.2
        global_vars.test_agent.target_victim_ = victim
        next_state = global_vars.test_agent.\
            all_states_["data_fusion_hold_state"].make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "validation_state")

    def test_exploration_mode_cost_function(self):
        rospy.loginfo('test_exploration_mode_cost_function')
        global_vars.test_agent.initial_time_ = rospy.get_rostime().secs - 600
        global_vars.test_agent.valid_victims_ = 1
        global_vars.test_agent.qrs_ = 10
        global_vars.test_agent.robot_resets_ = 0
        global_vars.test_agent.robot_restarts_ = 1
        cost_function = ExplorationModeCostFunction(global_vars.test_agent)
        cost = cost_function.execute()
        self.assertEqual(cost, 31.28940531542159)

    def test_exploration_mode_cost_function2(self):
        rospy.loginfo('test_exploration_mode_cost_function2')
        global_vars.test_agent.initial_time_ = rospy.get_rostime().secs - 600
        global_vars.test_agent.area_explored_ = 10
        global_vars.test_agent.valid_victims_ = 1
        global_vars.test_agent.qrs_ = 10
        global_vars.test_agent.robot_resets_ = 0
        global_vars.test_agent.robot_restarts_ = 1
        cost_function = ExplorationModeCostFunction2(global_vars.test_agent)
        cost = cost_function.execute()
        self.assertEqual(cost, 1.6729607187862237)

    def test_exploration_mode_cost_function3(self):
        rospy.loginfo('test_exploration_mode_cost_function3')
        global_vars.test_agent.initial_time_ = rospy.get_rostime().secs - 600
        global_vars.test_agent.area_explored_ = 10
        global_vars.test_agent.valid_victims_ = 1
        global_vars.test_agent.qrs_ = 10
        global_vars.test_agent.robot_resets_ = 0
        global_vars.test_agent.robot_restarts_ = 1
        cost_function = ExplorationModeCostFunction3(global_vars.test_agent)
        cost = cost_function.execute()
        self.assertEqual(cost, 1.9759825853101574)

    def test_exploration_strategy1_state_exploration(self):
        rospy.loginfo('test_exploration_strategy1_state_exploration')
        global_vars.test_agent.new_robot_state_ = robotModeMsg.MODE_EXPLORATION
        global_vars.test_agent.exploration_strategy_ = \
            "exploration_strategy1_state"
        global_vars.test_agent.define_states()
        global_vars.test_agent.initial_time_ = rospy.get_rostime().secs - 600
        global_vars.test_agent.valid_victims_ = 1
        global_vars.test_agent.qrs_ = 10
        global_vars.test_agent.robot_resets_ = 0
        global_vars.test_agent.robot_restarts_ = 1
        next_state = global_vars.test_agent.\
            all_states_["exploration_strategy1_state"].make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "exploration_strategy1_state")
        self.assertEqual(global_vars.test_agent.current_exploration_mode_,
                         DoExplorationGoal.TYPE_NORMAL)

    def test_exploration_strategy2_state_exploration(self):
        rospy.loginfo('test_exploration_strategy2_state_exploration')
        global_vars.test_agent.new_robot_state_ = robotModeMsg.MODE_EXPLORATION
        global_vars.test_agent.initial_time_ = rospy.get_rostime().secs - 600
        global_vars.test_agent.valid_victims_ = 2
        global_vars.test_agent.qrs_ = 10
        global_vars.test_agent.robot_resets_ = 1
        global_vars.test_agent.robot_restarts_ = 4
        next_state = global_vars.test_agent.\
            all_states_["exploration_strategy2_state"].make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "exploration_strategy2_state")
        self.assertEqual(global_vars.test_agent.current_exploration_mode_,
                         DoExplorationGoal.TYPE_FAST)

    def test_exploration_strategy2_state_identification(self):
        rospy.loginfo('test_exploration_strategy2_state_identification')
        global_vars.test_agent.new_robot_state_ = robotModeMsg.MODE_EXPLORATION
        global_vars.test_agent.current_robot_pose_.pose.position.x = 1.5
        global_vars.test_agent.current_robot_pose_.pose.position.y = 5.2
        victims = []
        victim = VictimInfoMsg()
        victim.id = 5
        victim.victimPose.pose.position.x = 3
        victim.victimPose.pose.position.y = 4
        victim.victimPose.pose.position.z = 3
        victim.probability = 0.2
        victims.append(victim)
        victim = VictimInfoMsg()
        victim.id = 8
        victim.victimPose.pose.position.x = 2
        victim.victimPose.pose.position.y = 6
        victim.victimPose.pose.position.z = 3.2
        victim.probability = 0.2
        victims.append(victim)
        global_vars.test_agent.new_victims_ = victims
        next_state = global_vars.test_agent.\
            all_states_["exploration_strategy2_state"].make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "identification_state")
        self.assertEqual(global_vars.test_agent.current_robot_state_,
                         robotModeMsg.MODE_IDENTIFICATION)

    def test_exploration_strategy2_state_orange_no_victims(self):
        rospy.loginfo('test_exploration_strategy2_state_orange_no_victims')
        global_vars.test_agent.new_robot_state_ = robotModeMsg.MODE_EXPLORATION
        global_vars.test_agent.current_arena_ = ArenaTypeMsg.TYPE_ORANGE
        next_state = global_vars.test_agent.\
            all_states_["exploration_strategy2_state"].make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "exploration_strategy2_state")
        self.assertEqual(global_vars.test_agent.current_exploration_mode_,
                         DoExplorationGoal.TYPE_FAST)

    def test_exploration_strategy2_state_orange_with_victims(self):
        rospy.loginfo('test_exploration_strategy2_state_orange_with_victims')
        global_vars.test_agent.new_robot_state_ = robotModeMsg.MODE_EXPLORATION
        global_vars.test_agent.current_arena_ = ArenaTypeMsg.TYPE_ORANGE
        global_vars.test_agent.valid_victims_ = 2
        next_state = global_vars.test_agent.\
            all_states_["exploration_strategy2_state"].make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "teleoperation_state")
        self.assertEqual(global_vars.test_agent.current_robot_state_,
                         robotModeMsg.MODE_TELEOPERATED_LOCOMOTION)

    def test_exploration_strategy2_state_teleoperation(self):
        rospy.loginfo('test_exploration_strategy2_state_teleoperation')
        global_vars.test_agent.new_robot_state_ = robotModeMsg.MODE_EXPLORATION
        global_vars.test_agent.transition_to_state(robotModeMsg.
                                                   MODE_TELEOPERATED_LOCOMOTION)
        rospy.Rate(10).sleep()
        next_state = global_vars.test_agent.\
            all_states_["exploration_strategy2_state"].make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "teleoperation_state")
        self.assertEqual(global_vars.test_agent.current_robot_state_,
                         robotModeMsg.MODE_TELEOPERATED_LOCOMOTION)

    def test_exploration_strategy3_state_exploration(self):
        rospy.loginfo('test_exploration_strategy3_state_exploration')
        global_vars.test_agent.new_robot_state_ = robotModeMsg.MODE_EXPLORATION
        global_vars.test_agent.exploration_strategy_ = \
            "exploration_strategy3_state"
        global_vars.test_agent.define_states()
        global_vars.test_agent.initial_time_ = rospy.get_rostime().secs - 600
        global_vars.test_agent.area_explored_ = 10
        global_vars.test_agent.valid_victims_ = 1
        global_vars.test_agent.qrs_ = 10
        global_vars.test_agent.robot_resets_ = 0
        global_vars.test_agent.robot_restarts_ = 1
        for i in range(10):
            global_vars.test_agent.deep_limit_ = \
                (1 + 0.135 - 0.003*global_vars.test_agent.max_time_/60) * \
                global_vars.test_agent.deep_limit_
        global_vars.test_agent.fast_limit_ = \
            global_vars.test_agent.deep_limit_ * 1.4
        next_state = global_vars.test_agent.\
            all_states_["exploration_strategy3_state"].make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "exploration_strategy3_state")
        self.assertEqual(global_vars.test_agent.current_exploration_mode_,
                         DoExplorationGoal.TYPE_NORMAL)

    def test_find_new_victim_to_go_cost_function(self):
        rospy.loginfo('test_find_new_victim_to_go_cost_function')
        global_vars.test_agent.current_robot_pose_.pose.position.x = 1.5
        global_vars.test_agent.current_robot_pose_.pose.position.y = 5.2
        victims = []
        victim = VictimInfoMsg()
        victim.id = 5
        victim.victimPose.pose.position.x = 3
        victim.victimPose.pose.position.y = 4
        victim.victimPose.pose.position.z = 3
        victim.probability = 0.2
        victims.append(victim)
        global_vars.test_agent.aborted_victims_.append([victim, 1])
        victim = VictimInfoMsg()
        victim.id = 8
        victim.victimPose.pose.position.x = 2
        victim.victimPose.pose.position.y = 6
        victim.victimPose.pose.position.z = 3.2
        victim.probability = 0.2
        victims.append(victim)
        global_vars.test_agent.new_victims_ = victims
        rospy.Rate(10).sleep()
        cost_function = FindNewVictimToGoCostFunction(global_vars.test_agent)
        cost = cost_function.execute()
        self.assertEqual(cost[0], -1.6046863561492728)
        self.assertEqual(cost[1], 5.283009433971698)

    def test_identification_state_aborted_victim_find_new_victim(self):
        rospy.loginfo('test_identification_state_\
                      aborted_victim_find_new_victim')
        global_vars.test_agent.new_robot_state_ = \
            robotModeMsg.MODE_IDENTIFICATION
        global_vars.test_agent.current_robot_pose_.pose.position.x = 1.8
        global_vars.test_agent.current_robot_pose_.pose.position.y = 8.6
        global_vars.com.robot_pose_.pose.position.x = 1.8
        global_vars.com.robot_pose_.pose.position.y = 8.6
        victims = []
        victim = VictimInfoMsg()
        victim.id = 9
        victim.victimPose.pose.position.x = 2
        victim.victimPose.pose.position.y = 9
        victim.victimPose.pose.position.z = 3.5
        victim.probability = 0.2
        victims.append(victim)
        global_vars.test_agent.new_victims_ = victims
        victim = VictimInfoMsg()
        victim.id = 8
        victim.victimPose.pose.position.x = 2
        victim.victimPose.pose.position.y = 6
        victim.victimPose.pose.position.z = 3.2
        victim.probability = 0.2
        global_vars.test_agent.target_victim_ = victim
        global_vars.test_agent.all_states_["identification_state"].execute()
        rospy.sleep(1.)
        global_vars.com.move_base_aborted_ = True
        rospy.sleep(1.)
        next_state = global_vars.test_agent.\
            all_states_["identification_state"].make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "identification_state")

    def test_identification_state_aborted_victim_return_to_exploration(self):
        rospy.loginfo('test_identification_state_\
                      aborted_victim_return_to_exploration')
        global_vars.test_agent.new_robot_state_ = \
            robotModeMsg.MODE_IDENTIFICATION
        global_vars.test_agent.current_robot_pose_.pose.position.x = 1.8
        global_vars.test_agent.current_robot_pose_.pose.position.y = 8.6
        global_vars.com.robot_pose_.pose.position.x = 1.8
        global_vars.com.robot_pose_.pose.position.y = 8.6
        victim = VictimInfoMsg()
        victim.id = 8
        victim.victimPose.pose.position.x = 2
        victim.victimPose.pose.position.y = 6
        victim.victimPose.pose.position.z = 3.2
        victim.probability = 0.2
        global_vars.test_agent.target_victim_ = victim
        global_vars.test_agent.all_states_["identification_state"].execute()
        rospy.sleep(1.)
        global_vars.com.move_base_aborted_ = True
        rospy.sleep(1.)
        next_state = global_vars.test_agent.\
            all_states_["identification_state"].make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "exploration_strategy2_state")

    def test_identification_state_succeeded_victim(self):
        rospy.loginfo('test_identification_state_succeeded_victim')
        global_vars.test_agent.new_robot_state_ = \
            robotModeMsg.MODE_IDENTIFICATION
        victims = []
        victim = VictimInfoMsg()
        victim.id = 8
        victim.victimPose.pose.position.x = 2
        victim.victimPose.pose.position.y = 6
        victim.victimPose.pose.position.z = 3.2
        victim.probability = 0.2
        victims.append(victim)
        global_vars.test_agent.new_victims_ = victims
        victim = VictimInfoMsg()
        victim.id = 8
        victim.victimPose.pose.position.x = 2
        victim.victimPose.pose.position.y = 6
        victim.victimPose.pose.position.z = 3.2
        victim.probability = 0.2
        global_vars.test_agent.target_victim_ = victim
        global_vars.test_agent.all_states_["identification_state"].execute()
        rospy.sleep(1.)
        global_vars.com.move_base_succeeded_ = True
        rospy.sleep(1.)
        next_state = global_vars.test_agent.\
            all_states_["identification_state"].make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "data_fusion_hold_state")
        self.assertEqual(global_vars.test_agent.current_robot_state_,
                         robotModeMsg.MODE_DF_HOLD)

    def test_identification_state_teleoperation(self):
        rospy.loginfo('test_identification_state_teleoperation')
        global_vars.test_agent.new_robot_state_ = \
            robotModeMsg.MODE_IDENTIFICATION
        global_vars.test_agent.transition_to_state(robotModeMsg.
                                                   MODE_TELEOPERATED_LOCOMOTION)
        rospy.Rate(10).sleep()
        next_state = global_vars.test_agent.\
            all_states_["identification_state"].make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "teleoperation_state")
        self.assertEqual(global_vars.test_agent.current_robot_state_,
                         robotModeMsg.MODE_TELEOPERATED_LOCOMOTION)

    def test_identification_state_update_victim(self):
        rospy.loginfo('test_identification_state_update_victim')
        global_vars.test_agent.new_robot_state_ = \
            robotModeMsg.MODE_IDENTIFICATION
        victims = []
        victim = VictimInfoMsg()
        victim.id = 8
        victim.victimPose.pose.position.x = 2
        victim.victimPose.pose.position.y = 6
        victim.victimPose.pose.position.z = 3.2
        victim.probability = 0.2
        victims.append(victim)
        global_vars.test_agent.new_victims_ = victims
        victim = VictimInfoMsg()
        victim.id = 8
        victim.victimPose.pose.position.x = 2
        victim.victimPose.pose.position.y = 6
        victim.victimPose.pose.position.z = 3.3
        victim.probability = 0.2
        global_vars.test_agent.target_victim_ = victim
        next_state = global_vars.test_agent.\
            all_states_["identification_state"].make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "identification_state")

    def test_qr_notification(self):
        rospy.loginfo('test_qr_notification')
        msg = QrNotificationMsg()
        global_vars.com.qr_notification_pub_.publish(msg)
        rospy.Rate(10).sleep()
        self.assertEqual(global_vars.test_agent.qrs_, 1)

    def test_reconfigure(self):
        rospy.loginfo('test_reconfigure')
        global_vars.com.dynamic_reconfigure_client.\
            update_configuration({"arenaVictims": 3, "maxQRs": 17,
                                  "arenaArea": 20, "maxTime": 1790,
                                  "timePassed": 0,
                                  "explorationStrategy": 1,
                                  "validVictimProbability": 0.7,
                                  "abortedVictimsDistance": 0.1,
                                  "deepLimit": 0.9})
        rospy.Rate(10).sleep()
        self.assertEqual(global_vars.test_agent.max_victims_, 3)
        self.assertEqual(global_vars.test_agent.max_qrs_, 17)
        self.assertEqual(global_vars.test_agent.max_area_, 20)
        self.assertEqual(global_vars.test_agent.max_time_, 1790)
        self.assertEqual(global_vars.test_agent.exploration_strategy_,
                         "exploration_strategy1_state")
        self.assertEqual(global_vars.test_agent.valid_victim_probability_, 0.7)
        self.assertEqual(global_vars.test_agent.aborted_victims_distance_, 0.1)
        self.assertEqual(global_vars.test_agent.deep_limit_, 0.9)

    def test_robocup_score(self):
        rospy.loginfo('test_robocup_score')
        msg = Int32(data=25)
        global_vars.com.robocup_score_pub_.publish(msg)
        rospy.Rate(10).sleep()
        self.assertEqual(global_vars.test_agent.current_score_, 25)

    def test_robot_reset(self):
        rospy.loginfo('test_robot_reset')
        global_vars.com.robot_reset_pub_.publish()
        rospy.Rate(10).sleep()
        self.assertEqual(global_vars.test_agent.robot_resets_, 1)

    def test_robot_restart(self):
        rospy.loginfo('test_robot_restart')
        global_vars.com.robot_restart_pub_.publish()
        rospy.Rate(10).sleep()
        self.assertEqual(global_vars.test_agent.robot_restarts_, 1)

    def test_robot_start_state_start_exploration(self):
        rospy.loginfo('test_robot_start_state_start_exploration')
        global_vars.test_agent.new_robot_state_ = \
            robotModeMsg.MODE_START_AUTONOMOUS
        global_vars.test_agent.all_states_["robot_start_state"].counter_ = 10
        next_state = global_vars.test_agent.\
            all_states_["robot_start_state"].make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "exploration_strategy2_state")
        self.assertEqual(global_vars.test_agent.current_robot_state_,
                         robotModeMsg.MODE_EXPLORATION)

    def test_robot_start_state_start_teleoperation(self):
        rospy.loginfo('test_robot_start_state_start_teleoperation')
        global_vars.test_agent.new_robot_state_ = \
            robotModeMsg.MODE_START_AUTONOMOUS
        global_vars.test_agent.transition_to_state(robotModeMsg.
                                                   MODE_TELEOPERATED_LOCOMOTION)
        rospy.Rate(10).sleep()
        next_state = global_vars.test_agent.\
            all_states_["robot_start_state"].make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "teleoperation_state")
        self.assertEqual(global_vars.test_agent.current_robot_state_,
                         robotModeMsg.MODE_TELEOPERATED_LOCOMOTION)

    def test_teleoperation_state_start_autonomous(self):
        rospy.loginfo('test_teleoperation_state_start_autonomous')
        global_vars.test_agent.new_robot_state_ = \
            robotModeMsg.MODE_TELEOPERATED_LOCOMOTION
        global_vars.test_agent.transition_to_state(robotModeMsg.
                                                   MODE_START_AUTONOMOUS)
        rospy.Rate(10).sleep()
        next_state = global_vars.test_agent.\
            all_states_["waiting_to_start_state"].make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "robot_start_state")
        self.assertEqual(global_vars.test_agent.current_robot_state_,
                         robotModeMsg.MODE_START_AUTONOMOUS)

    def test_update_victim_cost_function(self):
        rospy.loginfo('test_update_victim_cost_function')
        victims = []
        victim = VictimInfoMsg()
        victim.id = 3
        victim.victimPose.pose.position.x = 1
        victim.victimPose.pose.position.y = 2
        victim.victimPose.pose.position.z = 5.2
        victim.probability = 0.5
        victims.append(victim)
        global_vars.test_agent.new_victims_ = victims
        rospy.Rate(10).sleep()
        victim = VictimInfoMsg()
        victim.id = 3
        victim.victimPose.pose.position.x = 1
        victim.victimPose.pose.position.y = 2
        victim.victimPose.pose.position.z = 5.3
        victim.probability = 0.5
        global_vars.test_agent.target_victim_ = victim
        cost_function = UpdateVictimCostFunction(global_vars.test_agent)
        cost = cost_function.execute()
        self.assertEqual(cost, 1)

    def test_validation_state_execute(self):
        rospy.loginfo('test_validation_state_execute')
        global_vars.test_agent.all_states_["validation_state"].execute()
        rospy.Rate(10).sleep()
        self.assertEqual(global_vars.test_agent.valid_victims_, 1)

    def test_validation_state_exploration(self):
        rospy.loginfo('test_validation_state_exploration')
        global_vars.test_agent.new_robot_state_ = robotModeMsg.MODE_DF_HOLD
        global_vars.test_agent.current_robot_pose_.pose.position.x = 4.8
        global_vars.test_agent.current_robot_pose_.pose.position.y = 11.6
        victims = []
        victim = VictimInfoMsg()
        victim.id = 8
        victim.victimPose.pose.position.x = 2
        victim.victimPose.pose.position.y = 6
        victim.victimPose.pose.position.z = 3.5
        victim.probability = 0.3
        victims.append(victim)
        global_vars.test_agent.new_victims_ = victims
        victim = VictimInfoMsg()
        victim.id = 8
        victim.victimPose.pose.position.x = 2
        victim.victimPose.pose.position.y = 6
        victim.victimPose.pose.position.z = 3.5
        victim.probability = 0.2
        global_vars.test_agent.target_victim_ = victim
        next_state = global_vars.test_agent.\
            all_states_["validation_state"].make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "exploration_strategy2_state")
        self.assertEqual(global_vars.test_agent.current_robot_state_,
                         robotModeMsg.MODE_EXPLORATION)

    def test_validation_state_find_new_victim(self):
        rospy.loginfo('test_validation_state_find_new_victim')
        global_vars.test_agent.new_robot_state_ = robotModeMsg.MODE_DF_HOLD
        global_vars.test_agent.current_robot_pose_.pose.position.x = 1.8
        global_vars.test_agent.current_robot_pose_.pose.position.y = 8.6
        victims = []
        victim = VictimInfoMsg()
        victim.id = 8
        victim.victimPose.pose.position.x = 2
        victim.victimPose.pose.position.y = 6
        victim.victimPose.pose.position.z = 3.5
        victim.probability = 0.3
        victims.append(victim)
        victim = VictimInfoMsg()
        victim.id = 9
        victim.victimPose.pose.position.x = 2
        victim.victimPose.pose.position.y = 9
        victim.victimPose.pose.position.z = 3.2
        victim.probability = 0.2
        victims.append(victim)
        global_vars.test_agent.new_victims_ = victims
        victim = VictimInfoMsg()
        victim.id = 8
        victim.victimPose.pose.position.x = 2
        victim.victimPose.pose.position.y = 6
        victim.victimPose.pose.position.z = 3.5
        victim.probability = 0.2
        global_vars.test_agent.target_victim_ = victim
        next_state = global_vars.test_agent.\
            all_states_["validation_state"].make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "identification_state")
        self.assertEqual(global_vars.test_agent.current_robot_state_,
                         robotModeMsg.MODE_IDENTIFICATION)

    def test_validation_state_teleoperation(self):
        rospy.loginfo('test_validation_state_teleoperation')
        global_vars.test_agent.new_robot_state_ = robotModeMsg.MODE_DF_HOLD
        global_vars.test_agent.transition_to_state(robotModeMsg.
                                                   MODE_TELEOPERATED_LOCOMOTION)
        rospy.Rate(10).sleep()
        next_state = global_vars.test_agent.\
            all_states_["validation_state"].make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "teleoperation_state")
        self.assertEqual(global_vars.test_agent.current_robot_state_,
                         robotModeMsg.MODE_TELEOPERATED_LOCOMOTION)

    def test_victims_list(self):
        rospy.loginfo('test_victims_list')
        victims = []
        victim = VictimInfoMsg()
        victim.id = 5
        victim.victimPose.pose.position.x = 3
        victim.victimPose.pose.position.y = 4
        victim.probability = 0.2
        victims.append(victim)
        global_vars.com.victims_pub_.publish(victims)
        rospy.Rate(10).sleep()
        self.assertEqual(global_vars.test_agent.new_victims_[0].id, 5)
        self.assertEqual(global_vars.test_agent.new_victims_[0].
                         victimPose.pose.position.x, 3)
        self.assertEqual(global_vars.test_agent.new_victims_[0].
                         victimPose.pose.position.y, 4)
        self.assertAlmostEqual(global_vars.test_agent.new_victims_[0].
                               probability, 0.2, 5)

    def test_waiting_to_start_state_start_autonomous(self):
        rospy.loginfo('test_waiting_to_start_state_start_autonomous')
        global_vars.test_agent.transition_to_state(robotModeMsg.
                                                   MODE_START_AUTONOMOUS)
        rospy.Rate(10).sleep()
        next_state = global_vars.test_agent.\
            all_states_["waiting_to_start_state"].make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "robot_start_state")
        self.assertEqual(global_vars.test_agent.current_robot_state_,
                         robotModeMsg.MODE_START_AUTONOMOUS)

    def test_waiting_to_start_state_teleoperation(self):
        rospy.loginfo('test_waiting_to_start_state_teleoperation')
        global_vars.test_agent.transition_to_state(robotModeMsg.
                                                   MODE_TELEOPERATED_LOCOMOTION)
        rospy.Rate(10).sleep()
        next_state = global_vars.test_agent.\
            all_states_["waiting_to_start_state"].make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "teleoperation_state")
        self.assertEqual(global_vars.test_agent.current_robot_state_,
                         robotModeMsg.MODE_TELEOPERATED_LOCOMOTION)

if __name__ == '__main__':
    rospy.init_node('test_agent')
    global_vars.init()
    rospy.sleep(1.)
    suite = unittest.TestLoader().loadTestsFromTestCase(TestAgent)
    unittest.TextTestRunner(verbosity=1).run(suite)
    global_vars.com.delete_action_servers()
    rospy.spin()
