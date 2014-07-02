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

from pandora_fsm import agent_cost_functions

from state_manager_communications.msg import robotModeMsg
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Int32, Float32
from pandora_data_fusion_msgs.msg import WorldModelMsg, VictimInfoMsg, \
    QrNotificationMsg
from pandora_navigation_msgs.msg import ArenaTypeMsg, DoExplorationGoal


class TestAgent(unittest.TestCase):

    def tearDown(self):
        global_vars.com.robot_pose_ = PoseStamped()
        global_vars.test_agent.current_arena_ = ArenaTypeMsg.TYPE_YELLOW
        global_vars.test_agent.yellow_arena_area_explored_ = 0
        global_vars.test_agent.yellow_black_arena_area_explored_ = 0
        global_vars.test_agent.current_score_ = 0
        global_vars.test_agent.valid_victims_ = 0
        global_vars.test_agent.qrs_ = 0
        global_vars.test_agent.current_robot_pose_ = PoseStamped()
        global_vars.test_agent.current_exploration_mode_ = -1
        global_vars.test_agent.aborted_victims_ = []
        global_vars.test_agent.new_victims_ = []
        global_vars.test_agent.target_victim_ = VictimInfoMsg()

        global_vars.test_agent.strategy4_previous_victims_ = 0
        global_vars.test_agent.strategy4_previous_qrs_ = 0
        global_vars.test_agent.strategy4_previous_area_ = 0
        global_vars.test_agent.strategy4_previous_resets_ = 0
        global_vars.test_agent.strategy4_previous_restarts_ = 0
        global_vars.test_agent.strategy4_current_cost_ = 0

        global_vars.test_agent.current_robot_state_ = robotModeMsg.MODE_OFF

        global_vars.test_agent.current_state_ = \
            global_vars.test_agent.all_states_["waiting_to_start_state"]

        global_vars.com.dynamic_reconfigure_client.\
            update_configuration({"arenaVictims": 4,
                                  "maxQRs": 20,
                                  "yellowArenaArea": 18,
                                  "yellowBlackArenaArea": 12,
                                  "maxTime": 1200,
                                  "timePassed": 0,
                                  "validVictimProbability": 0.5,
                                  "abortedVictimsDistance": 0.5,
                                  "updatedVictimThreshold": 0.05,
                                  "abortedVictimSensorHold": 0.6,
                                  "robotResets": 0,
                                  "robotRestarts": 0,
                                  "explorationStrategy": 4,
                                  "strategy3DeepLimit": 0.85,
                                  "strategy4DeepLimit": 0,
                                  "strategy4FastLimit": 0.1,
                                  "strategy5DeepLimit": 0.85})

        global_vars.test_agent.strategy3_fast_limit_ = \
            global_vars.test_agent.strategy3_deep_limit_ * 1.4
        global_vars.test_agent.strategy5_fast_limit_ = \
            global_vars.test_agent.strategy5_deep_limit_ * 1.4

    def test_area_covered(self):
        rospy.loginfo('test_area_covered')
        msg = Float32(data=10)
        global_vars.com.area_covered_pub_.publish(msg)
        rospy.Rate(10).sleep()
        self.assertEqual(global_vars.test_agent.yellow_arena_area_explored_, 10)

    def test_arena_type(self):
        rospy.loginfo('test_arena_type')
        msg = ArenaTypeMsg(arena_type=ArenaTypeMsg.TYPE_ORANGE)
        global_vars.com.arena_type_pub_.publish(msg)
        rospy.Rate(10).sleep()
        self.assertEqual(global_vars.test_agent.current_arena_,
                         ArenaTypeMsg.TYPE_ORANGE)

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
        global_vars.test_agent.current_robot_state_ = robotModeMsg.MODE_DF_HOLD
        global_vars.test_agent.\
            all_states_["data_fusion_hold_state"].counter_ = 10
        global_vars.test_agent.current_robot_pose_.pose.position.x = 4.8
        global_vars.test_agent.current_robot_pose_.pose.position.y = 11.6
        victims_to_go = WorldModelMsg()
        victim = VictimInfoMsg()
        victim.id = 8
        victim.victimPose.pose.position.x = 2
        victim.victimPose.pose.position.y = 6
        victim.victimPose.pose.position.z = 3.5
        victim.victimPose.pose.orientation.x = 0.1
        victim.victimPose.pose.orientation.y = 0.5
        victim.victimPose.pose.orientation.z = 1
        victim.victimPose.pose.orientation.w = 0.9
        victim.probability = 0.3
        victims_to_go.victims.append(victim)
        global_vars.test_agent.new_victims_ = victims_to_go.victims
        victim = VictimInfoMsg()
        victim.id = 8
        victim.victimPose.pose.position.x = 2
        victim.victimPose.pose.position.y = 6
        victim.victimPose.pose.position.z = 3.5
        victim.victimPose.pose.orientation.x = 0.1
        victim.victimPose.pose.orientation.y = 0.5
        victim.victimPose.pose.orientation.z = 1
        victim.victimPose.pose.orientation.w = 0.9
        victim.probability = 0.2
        global_vars.test_agent.target_victim_ = victim
        next_state = global_vars.test_agent.\
            all_states_["data_fusion_hold_state"].make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "scan_end_effector_planner_state")
        self.assertEqual(global_vars.test_agent.current_robot_state_,
                         robotModeMsg.MODE_EXPLORATION)

    def test_data_fusion_hold_state_find_new_victim(self):
        rospy.loginfo('test_data_fusion_hold_state_find_new_victim')
        global_vars.test_agent.current_robot_state_ = robotModeMsg.MODE_DF_HOLD
        global_vars.test_agent.\
            all_states_["data_fusion_hold_state"].counter_ = 10
        global_vars.test_agent.current_robot_pose_.pose.position.x = 1.8
        global_vars.test_agent.current_robot_pose_.pose.position.y = 8.6
        victims_to_go = WorldModelMsg()
        victim = VictimInfoMsg()
        victim.id = 8
        victim.victimPose.pose.position.x = 2
        victim.victimPose.pose.position.y = 6
        victim.victimPose.pose.position.z = 3.5
        victim.victimPose.pose.orientation.x = 0.1
        victim.victimPose.pose.orientation.y = 0.5
        victim.victimPose.pose.orientation.z = 1
        victim.victimPose.pose.orientation.w = 0.9
        victim.probability = 0.3
        victims_to_go.victims.append(victim)
        victim = VictimInfoMsg()
        victim.id = 9
        victim.victimPose.pose.position.x = 2
        victim.victimPose.pose.position.y = 9
        victim.victimPose.pose.position.z = 3.2
        victim.victimPose.pose.orientation.x = 0.1
        victim.victimPose.pose.orientation.y = 0.5
        victim.victimPose.pose.orientation.z = 1
        victim.victimPose.pose.orientation.w = 0.9
        victim.probability = 0.2
        victims_to_go.victims.append(victim)
        global_vars.test_agent.new_victims_ = victims_to_go.victims
        victim = VictimInfoMsg()
        victim.id = 8
        victim.victimPose.pose.position.x = 2
        victim.victimPose.pose.position.y = 6
        victim.victimPose.pose.position.z = 3.5
        victim.victimPose.pose.orientation.x = 0.1
        victim.victimPose.pose.orientation.y = 0.5
        victim.victimPose.pose.orientation.z = 1
        victim.victimPose.pose.orientation.w = 0.9
        victim.probability = 0.2
        global_vars.test_agent.target_victim_ = victim
        next_state = global_vars.test_agent.\
            all_states_["data_fusion_hold_state"].make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "track_end_effector_planner_state")
        self.assertEqual(global_vars.test_agent.current_robot_state_,
                         robotModeMsg.MODE_IDENTIFICATION)

    def test_data_fusion_hold_state_stop_button(self):
        rospy.loginfo('test_data_fusion_hold_state_stop_button')
        global_vars.test_agent.current_robot_state_ = robotModeMsg.MODE_DF_HOLD
        global_vars.test_agent.transition_to_state(robotModeMsg.MODE_OFF)
        rospy.Rate(10).sleep()
        next_state = global_vars.test_agent.\
            all_states_["data_fusion_hold_state"].make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "stop_button_state")
        self.assertEqual(global_vars.test_agent.current_robot_state_,
                         robotModeMsg.MODE_OFF)

    def test_data_fusion_hold_state_teleoperation(self):
        rospy.loginfo('test_data_fusion_hold_state_teleoperation')
        global_vars.test_agent.current_robot_state_ = robotModeMsg.MODE_DF_HOLD
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
        global_vars.test_agent.current_robot_state_ = robotModeMsg.MODE_DF_HOLD
        global_vars.test_agent.\
            all_states_["data_fusion_hold_state"].counter_ = 10
        victims_to_go = WorldModelMsg()
        victim = VictimInfoMsg()
        victim.id = 8
        victim.victimPose.pose.position.x = 2
        victim.victimPose.pose.position.y = 6
        victim.victimPose.pose.position.z = 3.2
        victim.victimPose.pose.orientation.x = 0.1
        victim.victimPose.pose.orientation.y = 0.5
        victim.victimPose.pose.orientation.z = 1
        victim.victimPose.pose.orientation.w = 0.9
        victim.probability = 0.9
        victim.sensors.append('FACE')
        victim.sensors.append('THERMAL')
        victims_to_go.victims.append(victim)
        global_vars.test_agent.new_victims_ = victims_to_go.victims
        victim = VictimInfoMsg()
        victim.id = 8
        victim.victimPose.pose.position.x = 2
        victim.victimPose.pose.position.y = 6
        victim.victimPose.pose.position.z = 3.2
        victim.victimPose.pose.orientation.x = 0.1
        victim.victimPose.pose.orientation.y = 0.5
        victim.victimPose.pose.orientation.z = 1
        victim.victimPose.pose.orientation.w = 0.9
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
        cost_function = agent_cost_functions.exploration_mode_cost_function.\
            ExplorationModeCostFunction(global_vars.test_agent)
        cost = cost_function.execute()
        self.assertEqual(cost, 31.28940531542159)

    def test_exploration_mode_cost_function2(self):
        rospy.loginfo('test_exploration_mode_cost_function2')
        global_vars.test_agent.initial_time_ = rospy.get_rostime().secs - 600
        global_vars.test_agent.yellow_arena_area_explored_ = 10
        global_vars.test_agent.valid_victims_ = 1
        global_vars.test_agent.qrs_ = 10
        global_vars.test_agent.robot_resets_ = 0
        global_vars.test_agent.robot_restarts_ = 1
        cost_function = agent_cost_functions.exploration_mode2_cost_function.\
            ExplorationMode2CostFunction(global_vars.test_agent)
        cost = cost_function.execute()
        self.assertEqual(cost, 1.6729607187862237)

    def test_exploration_mode_cost_function3(self):
        rospy.loginfo('test_exploration_mode_cost_function3')
        global_vars.test_agent.initial_time_ = rospy.get_rostime().secs - 600
        global_vars.test_agent.yellow_arena_area_explored_ = 10
        global_vars.test_agent.valid_victims_ = 1
        global_vars.test_agent.qrs_ = 10
        global_vars.test_agent.robot_resets_ = 0
        global_vars.test_agent.robot_restarts_ = 1
        cost_function = agent_cost_functions.exploration_mode3_cost_function.\
            ExplorationMode3CostFunction(global_vars.test_agent)
        cost = cost_function.execute()
        self.assertEqual(cost, 1.9759825853101574)

    def test_exploration_mode_cost_function4(self):
        rospy.loginfo('test_exploration_mode_cost_function4')
        global_vars.test_agent.initial_time_ = rospy.get_rostime().secs - 600
        global_vars.test_agent.yellow_arena_area_explored_ = 10
        global_vars.test_agent.valid_victims_ = 1
        global_vars.test_agent.qrs_ = 10
        global_vars.test_agent.robot_resets_ = 0
        global_vars.test_agent.robot_restarts_ = 1
        cost_function = agent_cost_functions.exploration_mode4_cost_function.\
            ExplorationMode4CostFunction(global_vars.test_agent)
        cost = cost_function.execute()
        self.assertEqual(cost, -0.06116320402034687)

    def test_exploration_strategy1_state_exploration(self):
        rospy.loginfo('test_exploration_strategy1_state_exploration')
        global_vars.test_agent.current_robot_state_ = \
            robotModeMsg.MODE_EXPLORATION
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
        self.assertEqual(next_state,
                         global_vars.test_agent.exploration_strategy_)
        self.assertEqual(global_vars.test_agent.current_exploration_mode_,
                         DoExplorationGoal.TYPE_NORMAL)

    def test_exploration_strategy2_state_exploration(self):
        rospy.loginfo('test_exploration_strategy2_state_exploration')
        global_vars.test_agent.current_robot_state_ = \
            robotModeMsg.MODE_EXPLORATION
        global_vars.test_agent.initial_time_ = rospy.get_rostime().secs - 600
        global_vars.test_agent.yellow_arena_area_explored_ = 10
        global_vars.test_agent.valid_victims_ = 2
        global_vars.test_agent.qrs_ = 10
        global_vars.test_agent.robot_resets_ = 1
        global_vars.test_agent.robot_restarts_ = 4
        next_state = global_vars.test_agent.\
            all_states_["exploration_strategy2_state"].make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state,
                         global_vars.test_agent.exploration_strategy_)
        self.assertEqual(global_vars.test_agent.current_exploration_mode_,
                         DoExplorationGoal.TYPE_FAST)

    def test_exploration_strategy3_state_exploration(self):
        rospy.loginfo('test_exploration_strategy3_state_exploration')
        global_vars.test_agent.current_robot_state_ = \
            robotModeMsg.MODE_EXPLORATION
        global_vars.test_agent.exploration_strategy_ = \
            "exploration_strategy3_state"
        global_vars.test_agent.define_states()
        global_vars.test_agent.initial_time_ = rospy.get_rostime().secs - 600
        global_vars.test_agent.yellow_arena_area_explored_ = 10
        global_vars.test_agent.valid_victims_ = 1
        global_vars.test_agent.qrs_ = 10
        global_vars.test_agent.robot_resets_ = 0
        global_vars.test_agent.robot_restarts_ = 1
        for i in range(10):
            global_vars.test_agent.minutes_passed_ += 60
            global_vars.test_agent.strategy3_deep_limit_ = \
                (1 + 0.135 - 0.003*global_vars.test_agent.max_time_/60) * \
                global_vars.test_agent.strategy3_deep_limit_
        global_vars.test_agent.strategy3_fast_limit_ = \
            global_vars.test_agent.strategy3_deep_limit_ * 1.4
        next_state = global_vars.test_agent.\
            all_states_["exploration_strategy3_state"].make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state,
                         global_vars.test_agent.exploration_strategy_)
        self.assertEqual(global_vars.test_agent.current_exploration_mode_,
                         DoExplorationGoal.TYPE_NORMAL)

    def test_exploration_strategy4_state_exploration(self):
        rospy.loginfo('test_exploration_strategy4_state_exploration')
        global_vars.test_agent.current_robot_state_ = \
            robotModeMsg.MODE_EXPLORATION
        global_vars.test_agent.initial_time_ = rospy.get_rostime().secs - 600
        global_vars.test_agent.yellow_arena_area_explored_ = 10
        global_vars.test_agent.valid_victims_ = 2
        global_vars.test_agent.qrs_ = 10
        global_vars.test_agent.robot_resets_ = 1
        global_vars.test_agent.robot_restarts_ = 4
        next_state = global_vars.test_agent.\
            all_states_["exploration_strategy4_state"].make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state,
                         global_vars.test_agent.exploration_strategy_)
        self.assertEqual(global_vars.test_agent.current_exploration_mode_,
                         DoExplorationGoal.TYPE_NORMAL)

    def test_exploration_strategy4_state_identification(self):
        rospy.loginfo('test_exploration_strategy4_state_identification')
        global_vars.test_agent.current_robot_state_ = \
            robotModeMsg.MODE_EXPLORATION
        global_vars.test_agent.current_robot_pose_.pose.position.x = 1.5
        global_vars.test_agent.current_robot_pose_.pose.position.y = 5.2
        victims_to_go = WorldModelMsg()
        victim = VictimInfoMsg()
        victim.id = 5
        victim.victimPose.pose.position.x = 3
        victim.victimPose.pose.position.y = 4
        victim.victimPose.pose.position.z = 3
        victim.victimPose.pose.orientation.x = 0.1
        victim.victimPose.pose.orientation.y = 0.5
        victim.victimPose.pose.orientation.z = 1
        victim.victimPose.pose.orientation.w = 0.9
        victim.probability = 0.2
        victims_to_go.victims.append(victim)
        victim = VictimInfoMsg()
        victim.id = 8
        victim.victimPose.pose.position.x = 2
        victim.victimPose.pose.position.y = 6
        victim.victimPose.pose.position.z = 3.2
        victim.victimPose.pose.orientation.x = 0.1
        victim.victimPose.pose.orientation.y = 0.5
        victim.victimPose.pose.orientation.z = 1
        victim.victimPose.pose.orientation.w = 0.9
        victim.probability = 0.2
        victims_to_go.victims.append(victim)
        global_vars.test_agent.new_victims_ = victims_to_go.victims
        next_state = global_vars.test_agent.\
            all_states_["exploration_strategy4_state"].make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "track_end_effector_planner_state")
        self.assertEqual(global_vars.test_agent.current_robot_state_,
                         robotModeMsg.MODE_IDENTIFICATION)

    def test_exploration_strategy4_state_orange_no_victims(self):
        rospy.loginfo('test_exploration_strategy4_state_orange_no_victims')
        global_vars.test_agent.current_robot_state_ = \
            robotModeMsg.MODE_EXPLORATION
        global_vars.test_agent.current_arena_ = ArenaTypeMsg.TYPE_ORANGE
        next_state = global_vars.test_agent.\
            all_states_["exploration_strategy4_state"].make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state,
                         global_vars.test_agent.exploration_strategy_)
        self.assertEqual(global_vars.test_agent.current_exploration_mode_,
                         DoExplorationGoal.TYPE_FAST)

    def test_exploration_strategy4_state_orange_with_victims(self):
        rospy.loginfo('test_exploration_strategy4_state_orange_with_victims')
        global_vars.test_agent.current_robot_state_ = \
            robotModeMsg.MODE_EXPLORATION
        global_vars.test_agent.current_arena_ = ArenaTypeMsg.TYPE_ORANGE
        global_vars.test_agent.valid_victims_ = 2
        next_state = global_vars.test_agent.\
            all_states_["exploration_strategy4_state"].make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "teleoperation_state")
        self.assertEqual(global_vars.test_agent.current_robot_state_,
                         robotModeMsg.MODE_TELEOPERATED_LOCOMOTION)

    def test_exploration_strategy4_state_stop_button(self):
        rospy.loginfo('test_exploration_strategy4_state_stop_button')
        global_vars.test_agent.current_robot_state_ = \
            robotModeMsg.MODE_EXPLORATION
        global_vars.test_agent.transition_to_state(robotModeMsg.MODE_OFF)
        rospy.Rate(10).sleep()
        next_state = global_vars.test_agent.\
            all_states_["exploration_strategy4_state"].make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "stop_button_state")
        self.assertEqual(global_vars.test_agent.current_robot_state_,
                         robotModeMsg.MODE_OFF)

    def test_exploration_strategy4_state_teleoperation(self):
        rospy.loginfo('test_exploration_strategy4_state_teleoperation')
        global_vars.test_agent.current_robot_state_ = \
            robotModeMsg.MODE_EXPLORATION
        global_vars.test_agent.transition_to_state(robotModeMsg.
                                                   MODE_TELEOPERATED_LOCOMOTION)
        rospy.Rate(10).sleep()
        next_state = global_vars.test_agent.\
            all_states_["exploration_strategy4_state"].make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "teleoperation_state")
        self.assertEqual(global_vars.test_agent.current_robot_state_,
                         robotModeMsg.MODE_TELEOPERATED_LOCOMOTION)

    def test_exploration_strategy5_state_exploration(self):
        rospy.loginfo('test_exploration_strategy5_state_exploration')
        global_vars.test_agent.current_robot_state_ = \
            robotModeMsg.MODE_EXPLORATION
        global_vars.test_agent.exploration_strategy_ = \
            "exploration_strategy5_state"
        global_vars.test_agent.define_states()
        global_vars.test_agent.initial_time_ = rospy.get_rostime().secs - 600
        global_vars.test_agent.yellow_arena_area_explored_ = 10
        global_vars.test_agent.valid_victims_ = 1
        global_vars.test_agent.qrs_ = 10
        global_vars.test_agent.robot_resets_ = 0
        global_vars.test_agent.robot_restarts_ = 1
        for i in range(10):
            global_vars.test_agent.minutes_passed_ += 60
            global_vars.test_agent.strategy5_deep_limit_ = \
                (1 + 0.135 - 0.003*global_vars.test_agent.max_time_/60) * \
                global_vars.test_agent.strategy5_deep_limit_
        global_vars.test_agent.strategy5_fast_limit_ = \
            global_vars.test_agent.strategy5_deep_limit_ * 1.4
        next_state = global_vars.test_agent.\
            all_states_["exploration_strategy5_state"].make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state,
                         global_vars.test_agent.exploration_strategy_)
        self.assertEqual(global_vars.test_agent.current_exploration_mode_,
                         DoExplorationGoal.TYPE_NORMAL)

    def test_find_new_victim_to_go_cost_function(self):
        rospy.loginfo('test_find_new_victim_to_go_cost_function')
        global_vars.test_agent.current_robot_pose_.pose.position.x = 1.5
        global_vars.test_agent.current_robot_pose_.pose.position.y = 5.2
        victims_to_go = WorldModelMsg()
        victim = VictimInfoMsg()
        victim.id = 5
        victim.victimPose.pose.position.x = 3
        victim.victimPose.pose.position.y = 4
        victim.victimPose.pose.position.z = 3
        victim.victimPose.pose.orientation.x = 0.1
        victim.victimPose.pose.orientation.y = 0.5
        victim.victimPose.pose.orientation.z = 1
        victim.victimPose.pose.orientation.w = 0.9
        victim.probability = 0.2
        victims_to_go.victims.append(victim)
        global_vars.test_agent.aborted_victims_.append([victim, 1])
        victim = VictimInfoMsg()
        victim.id = 8
        victim.victimPose.pose.position.x = 2
        victim.victimPose.pose.position.y = 6
        victim.victimPose.pose.position.z = 3.2
        victim.victimPose.pose.orientation.x = 0.1
        victim.victimPose.pose.orientation.y = 0.5
        victim.victimPose.pose.orientation.z = 1
        victim.victimPose.pose.orientation.w = 0.9
        victim.probability = 0.2
        victims_to_go.victims.append(victim)
        global_vars.test_agent.new_victims_ = victims_to_go.victims
        rospy.Rate(10).sleep()
        cost_function = \
            agent_cost_functions.find_new_victim_to_go_cost_function.\
            FindNewVictimToGoCostFunction(global_vars.test_agent)
        cost = cost_function.execute()
        self.assertEqual(cost[0], -1.6046863561492728)
        self.assertEqual(cost[1], 5.283009433971698)

    def test_identification_move_to_victim_state_identification(self):
        rospy.loginfo('test_identification_move_to_victim_state_identification')
        global_vars.test_agent.current_robot_state_ = \
            robotModeMsg.MODE_IDENTIFICATION
        next_state = global_vars.test_agent.\
            all_states_["identification_move_to_victim_state"].\
            make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, 'identification_check_for_victims_state')
        self.assertEqual(global_vars.test_agent.current_robot_state_,
                         robotModeMsg.MODE_IDENTIFICATION)

    def test_identification_move_to_victim_state_teleoperation(self):
        rospy.loginfo('test_identification_move_to_victim_state_teleoperation')
        global_vars.test_agent.transition_to_state(robotModeMsg.
                                                   MODE_TELEOPERATED_LOCOMOTION)
        rospy.Rate(10).sleep()
        next_state = global_vars.test_agent.\
            all_states_["identification_move_to_victim_state"].\
            make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "teleoperation_state")
        self.assertEqual(global_vars.test_agent.current_robot_state_,
                         robotModeMsg.MODE_TELEOPERATED_LOCOMOTION)

    def test_identification_state_aborted_victim_find_new_victim(self):
        rospy.loginfo('test_identification_state_\
                      aborted_victim_find_new_victim')
        global_vars.test_agent.current_robot_state_ = \
            robotModeMsg.MODE_IDENTIFICATION
        global_vars.test_agent.current_robot_pose_.pose.position.x = 1.8
        global_vars.test_agent.current_robot_pose_.pose.position.y = 8.6
        global_vars.com.robot_pose_.pose.position.x = 1.8
        global_vars.com.robot_pose_.pose.position.y = 8.6
        victims_to_go = WorldModelMsg()
        victim = VictimInfoMsg()
        victim.id = 9
        victim.victimPose.pose.position.x = 2
        victim.victimPose.pose.position.y = 9
        victim.victimPose.pose.position.z = 3.5
        victim.victimPose.pose.orientation.x = 0.1
        victim.victimPose.pose.orientation.y = 0.5
        victim.victimPose.pose.orientation.z = 1
        victim.victimPose.pose.orientation.w = 0.9
        victim.probability = 0.2
        victims_to_go.victims.append(victim)
        global_vars.test_agent.new_victims_ = victims_to_go.victims
        victim = VictimInfoMsg()
        victim.id = 8
        victim.victimPose.pose.position.x = 2
        victim.victimPose.pose.position.y = 6
        victim.victimPose.pose.position.z = 3.2
        victim.victimPose.pose.orientation.x = 0.1
        victim.victimPose.pose.orientation.y = 0.5
        victim.victimPose.pose.orientation.z = 1
        victim.victimPose.pose.orientation.w = 0.9
        victim.probability = 0.2
        global_vars.test_agent.target_victim_ = victim
        global_vars.test_agent.\
            all_states_["identification_move_to_victim_state"].execute()
        rospy.sleep(1.)
        global_vars.com.move_base_aborted_ = True
        rospy.sleep(1.)
        next_state = global_vars.test_agent.\
            all_states_["identification_check_for_victims_state"].\
            make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "track_end_effector_planner_state")

    def test_identification_state_aborted_victim_return_to_exploration(self):
        rospy.loginfo('test_identification_state_\
                      aborted_victim_return_to_exploration')
        global_vars.test_agent.current_robot_state_ = \
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
        victim.victimPose.pose.orientation.x = 0.1
        victim.victimPose.pose.orientation.y = 0.5
        victim.victimPose.pose.orientation.z = 1
        victim.victimPose.pose.orientation.w = 0.9
        victim.probability = 0.2
        global_vars.test_agent.target_victim_ = victim
        global_vars.test_agent.\
            all_states_["identification_move_to_victim_state"].execute()
        rospy.sleep(1.)
        global_vars.com.move_base_aborted_ = True
        rospy.sleep(1.)
        next_state = global_vars.test_agent.\
            all_states_["identification_check_for_victims_state"].\
            make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "scan_end_effector_planner_state")

    def test_identification_state_stop_button(self):
        rospy.loginfo('test_identification_state_stop_button')
        global_vars.test_agent.current_robot_state_ = \
            robotModeMsg.MODE_IDENTIFICATION
        global_vars.test_agent.transition_to_state(robotModeMsg.MODE_OFF)
        rospy.Rate(10).sleep()
        next_state = global_vars.test_agent.\
            all_states_["identification_check_for_victims_state"].\
            make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "stop_button_state")
        self.assertEqual(global_vars.test_agent.current_robot_state_,
                         robotModeMsg.MODE_OFF)

    def test_identification_state_succeeded_victim(self):
        rospy.loginfo('test_identification_state_succeeded_victim')
        global_vars.test_agent.current_robot_state_ = \
            robotModeMsg.MODE_IDENTIFICATION
        victims_to_go = WorldModelMsg()
        victim = VictimInfoMsg()
        victim.id = 8
        victim.victimPose.pose.position.x = 2
        victim.victimPose.pose.position.y = 6
        victim.victimPose.pose.position.z = 3.2
        victim.victimPose.pose.orientation.x = 0.1
        victim.victimPose.pose.orientation.y = 0.5
        victim.victimPose.pose.orientation.z = 1
        victim.victimPose.pose.orientation.w = 0.9
        victim.probability = 0.2
        victims_to_go.victims.append(victim)
        global_vars.test_agent.new_victims_ = victims_to_go.victims
        victim = VictimInfoMsg()
        victim.id = 8
        victim.victimPose.pose.position.x = 2
        victim.victimPose.pose.position.y = 6
        victim.victimPose.pose.position.z = 3.2
        victim.victimPose.pose.orientation.x = 0.1
        victim.victimPose.pose.orientation.y = 0.5
        victim.victimPose.pose.orientation.z = 1
        victim.victimPose.pose.orientation.w = 0.9
        victim.probability = 0.2
        global_vars.test_agent.target_victim_ = victim
        global_vars.test_agent.\
            all_states_["identification_move_to_victim_state"].execute()
        rospy.sleep(1.)
        global_vars.com.move_base_succeeded_ = True
        rospy.sleep(1.)
        next_state = global_vars.test_agent.\
            all_states_["identification_check_for_victims_state"].\
            make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "lax_track_end_effector_planner_state")

    def test_identification_state_teleoperation(self):
        rospy.loginfo('test_identification_state_teleoperation')
        global_vars.test_agent.current_robot_state_ = \
            robotModeMsg.MODE_IDENTIFICATION
        global_vars.test_agent.transition_to_state(robotModeMsg.
                                                   MODE_TELEOPERATED_LOCOMOTION)
        rospy.Rate(10).sleep()
        next_state = global_vars.test_agent.\
            all_states_["identification_check_for_victims_state"].\
            make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "teleoperation_state")
        self.assertEqual(global_vars.test_agent.current_robot_state_,
                         robotModeMsg.MODE_TELEOPERATED_LOCOMOTION)

    def test_identification_state_update_victim(self):
        rospy.loginfo('test_identification_state_update_victim')
        global_vars.test_agent.current_robot_state_ = \
            robotModeMsg.MODE_IDENTIFICATION
        victims_to_go = WorldModelMsg()
        victim = VictimInfoMsg()
        victim.id = 8
        victim.victimPose.pose.position.x = 2
        victim.victimPose.pose.position.y = 6
        victim.victimPose.pose.position.z = 3.2
        victim.victimPose.pose.orientation.x = 0.1
        victim.victimPose.pose.orientation.y = 0.5
        victim.victimPose.pose.orientation.z = 1
        victim.victimPose.pose.orientation.w = 0.9
        victim.probability = 0.2
        victims_to_go.victims.append(victim)
        global_vars.test_agent.new_victims_ = victims_to_go.victims
        victim = VictimInfoMsg()
        victim.id = 8
        victim.victimPose.pose.position.x = 2
        victim.victimPose.pose.position.y = 6.1
        victim.victimPose.pose.position.z = 3.2
        victim.victimPose.pose.orientation.x = 0.1
        victim.victimPose.pose.orientation.y = 0.5
        victim.victimPose.pose.orientation.z = 1
        victim.victimPose.pose.orientation.w = 0.9
        victim.probability = 0.2
        global_vars.test_agent.target_victim_ = victim
        next_state = global_vars.test_agent.\
            all_states_["identification_check_for_victims_state"].\
            make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "track_end_effector_planner_state")

    def test_lax_track_end_effector_planner_state_data_fusion_hold(self):
        rospy.loginfo('test_lax_track_end_effector_planner_state_\
                      data_fusion_hold')
        global_vars.test_agent.current_robot_state_ = \
            robotModeMsg.MODE_IDENTIFICATION
        next_state = global_vars.test_agent.\
            all_states_["lax_track_end_effector_planner_state"].\
            make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, 'data_fusion_hold_state')

    def test_lax_track_end_effector_planner_state_teleoperation(self):
        rospy.loginfo('test_lax_track_end_effector_planner_state_teleoperation')
        global_vars.test_agent.transition_to_state(robotModeMsg.
                                                   MODE_TELEOPERATED_LOCOMOTION)
        rospy.Rate(10).sleep()
        next_state = global_vars.test_agent.\
            all_states_["lax_track_end_effector_planner_state"].\
            make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "teleoperation_state")
        self.assertEqual(global_vars.test_agent.current_robot_state_,
                         robotModeMsg.MODE_TELEOPERATED_LOCOMOTION)

    def test_qr_notification(self):
        rospy.loginfo('test_qr_notification')
        msg = QrNotificationMsg()
        global_vars.com.qr_notification_pub_.publish(msg)
        rospy.Rate(10).sleep()
        self.assertEqual(global_vars.test_agent.qrs_, 1)

    def test_reconfigure(self):
        rospy.loginfo('test_reconfigure')
        global_vars.com.dynamic_reconfigure_client.\
            update_configuration({"arenaVictims": 3,
                                  "maxQRs": 17,
                                  "yellowArenaArea": 20,
                                  "yellowBlackArenaArea": 15,
                                  "maxTime": 1790,
                                  "timePassed": 0,
                                  "validVictimProbability": 0.7,
                                  "abortedVictimsDistance": 0.1,
                                  "updatedVictimThreshold": 0.1,
                                  "abortedVictimSensorHold": 0.8,
                                  "robotResets": 1,
                                  "robotRestarts": 3,
                                  "explorationStrategy": 1,
                                  "arenaType": 1,
                                  "strategy3DeepLimit": 0.9,
                                  "strategy4DeepLimit": 0.1,
                                  "strategy4FastLimit": 0.2,
                                  "strategy5DeepLimit": 0.95})
        rospy.Rate(10).sleep()
        self.assertEqual(global_vars.test_agent.max_victims_, 3)
        self.assertEqual(global_vars.test_agent.max_qrs_, 17)
        self.assertEqual(global_vars.test_agent.max_yellow_area_, 20)
        self.assertEqual(global_vars.test_agent.max_yellow_black_area_, 15)
        self.assertEqual(global_vars.test_agent.max_time_, 1790)
        self.assertEqual(global_vars.test_agent.valid_victim_probability_, 0.7)
        self.assertEqual(global_vars.test_agent.aborted_victims_distance_, 0.1)
        self.assertEqual(global_vars.test_agent.updated_victim_threshold_, 0.1)
        self.assertEqual(global_vars.test_agent.
                         aborted_victim_sensor_hold_, 0.8)
        self.assertEqual(global_vars.test_agent.robot_resets_, 1)
        self.assertEqual(global_vars.test_agent.robot_restarts_, 3)
        self.assertEqual(global_vars.test_agent.exploration_strategy_,
                         "yellow_black_arena_save_robot_pose_state")
        self.assertEqual(global_vars.test_agent.strategy3_deep_limit_, 0.9)
        self.assertEqual(global_vars.test_agent.strategy4_deep_limit_, 0.1)
        self.assertEqual(global_vars.test_agent.strategy4_fast_limit_, 0.2)
        self.assertEqual(global_vars.test_agent.strategy5_deep_limit_, 0.95)

    def test_robocup_score(self):
        rospy.loginfo('test_robocup_score')
        msg = Int32(data=25)
        global_vars.com.robocup_score_pub_.publish(msg)
        rospy.Rate(10).sleep()
        self.assertEqual(global_vars.test_agent.current_score_, 25)

    def test_robot_start_state_start_exploration(self):
        rospy.loginfo('test_robot_start_state_start_exploration')
        global_vars.test_agent.current_robot_state_ = \
            robotModeMsg.MODE_START_AUTONOMOUS
        global_vars.test_agent.all_states_["robot_start_state"].counter_ = 10
        next_state = global_vars.test_agent.\
            all_states_["robot_start_state"].make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "scan_end_effector_planner_state")
        self.assertEqual(global_vars.test_agent.current_robot_state_,
                         robotModeMsg.MODE_EXPLORATION)

    def test_robot_start_state_start_stop_button(self):
        rospy.loginfo('test_robot_start_state_start_stop_button')
        global_vars.test_agent.current_robot_state_ = \
            robotModeMsg.MODE_START_AUTONOMOUS
        global_vars.test_agent.transition_to_state(robotModeMsg.MODE_OFF)
        rospy.Rate(10).sleep()
        next_state = global_vars.test_agent.\
            all_states_["robot_start_state"].make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "stop_button_state")
        self.assertEqual(global_vars.test_agent.current_robot_state_,
                         robotModeMsg.MODE_OFF)

    def test_robot_start_state_start_teleoperation(self):
        rospy.loginfo('test_robot_start_state_start_teleoperation')
        global_vars.test_agent.current_robot_state_ = \
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

    def test_scan_end_effector_planner_state_exploration(self):
        rospy.loginfo('test_scan_end_effector_planner_state_exploration')
        global_vars.test_agent.current_robot_state_ = \
            robotModeMsg.MODE_EXPLORATION
        next_state = global_vars.test_agent.\
            all_states_["scan_end_effector_planner_state"].\
            make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state,
                         global_vars.test_agent.exploration_strategy_)

    def test_scan_end_effector_planner_state_teleoperation(self):
        rospy.loginfo('test_scan_end_effector_planner_state_teleoperation')
        global_vars.test_agent.transition_to_state(robotModeMsg.
                                                   MODE_TELEOPERATED_LOCOMOTION)
        rospy.Rate(10).sleep()
        next_state = global_vars.test_agent.\
            all_states_["scan_end_effector_planner_state"].\
            make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "teleoperation_state")
        self.assertEqual(global_vars.test_agent.current_robot_state_,
                         robotModeMsg.MODE_TELEOPERATED_LOCOMOTION)

    def test_teleoperation_state_start_autonomous(self):
        rospy.loginfo('test_teleoperation_state_start_autonomous')
        global_vars.test_agent.current_robot_state_ = \
            robotModeMsg.MODE_TELEOPERATED_LOCOMOTION
        global_vars.test_agent.transition_to_state(robotModeMsg.
                                                   MODE_START_AUTONOMOUS)
        rospy.Rate(10).sleep()
        next_state = global_vars.test_agent.\
            all_states_["teleoperation_state"].make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "scan_end_effector_planner_state")
        self.assertEqual(global_vars.test_agent.current_robot_state_,
                         robotModeMsg.MODE_EXPLORATION)

    def test_test_and_park_end_effector_planner_state_robot_start(self):
        rospy.loginfo('test_test_and_park_end_effector_planner_state_\
                      robot_start')
        global_vars.test_agent.current_robot_state_ = \
            robotModeMsg.MODE_START_AUTONOMOUS
        next_state = global_vars.test_agent.\
            all_states_["test_and_park_end_effector_planner_state"].\
            make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "robot_start_state")
        self.assertEqual(global_vars.test_agent.current_robot_state_,
                         robotModeMsg.MODE_START_AUTONOMOUS)

    def test_test_and_park_end_effector_planner_state_teleoperation(self):
        rospy.loginfo('test_test_and_park_end_effector_planner_state_\
                      teleoperation')
        global_vars.test_agent.transition_to_state(robotModeMsg.
                                                   MODE_TELEOPERATED_LOCOMOTION)
        rospy.Rate(10).sleep()
        next_state = global_vars.test_agent.\
            all_states_["test_and_park_end_effector_planner_state"].\
            make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "teleoperation_state")
        self.assertEqual(global_vars.test_agent.current_robot_state_,
                         robotModeMsg.MODE_TELEOPERATED_LOCOMOTION)

    def test_test_and_park_end_effector_planner_state_waiting_to_start(self):
        rospy.loginfo('test_test_and_park_end_effector_planner_state_\
                      waiting_to_start')
        global_vars.test_agent.current_robot_state_ = \
            robotModeMsg.MODE_START_AUTONOMOUS
        global_vars.com.move_end_effector_succeeded_ = True
        global_vars.com.move_end_effector_aborted_ = True
        global_vars.test_agent.\
            all_states_["test_and_park_end_effector_planner_state"].execute()
        next_state = global_vars.test_agent.\
            all_states_["test_and_park_end_effector_planner_state"].\
            make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "waiting_to_start_state")
        self.assertEqual(global_vars.test_agent.current_robot_state_,
                         robotModeMsg.MODE_OFF)

    def test_track_end_effector_planner_state_identification(self):
        rospy.loginfo('test_track_end_effector_planner_state_identification')
        global_vars.test_agent.current_robot_state_ = \
            robotModeMsg.MODE_IDENTIFICATION
        next_state = global_vars.test_agent.\
            all_states_["track_end_effector_planner_state"].\
            make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, 'identification_move_to_victim_state')

    def test_track_end_effector_planner_state_teleoperation(self):
        rospy.loginfo('test_track_end_effector_planner_state_teleoperation')
        global_vars.test_agent.transition_to_state(robotModeMsg.
                                                   MODE_TELEOPERATED_LOCOMOTION)
        rospy.Rate(10).sleep()
        next_state = global_vars.test_agent.\
            all_states_["track_end_effector_planner_state"].\
            make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "teleoperation_state")
        self.assertEqual(global_vars.test_agent.current_robot_state_,
                         robotModeMsg.MODE_TELEOPERATED_LOCOMOTION)

    def test_update_victim_cost_function(self):
        rospy.loginfo('test_update_victim_cost_function')
        victims_to_go = WorldModelMsg()
        victim = VictimInfoMsg()
        victim.id = 3
        victim.victimPose.pose.position.x = 1
        victim.victimPose.pose.position.y = 2
        victim.victimPose.pose.position.z = 5.2
        victim.victimPose.pose.orientation.x = 0.1
        victim.victimPose.pose.orientation.y = 0.5
        victim.victimPose.pose.orientation.z = 1
        victim.victimPose.pose.orientation.w = 0.9
        victim.probability = 0.5
        victims_to_go.victims.append(victim)
        global_vars.test_agent.new_victims_ = victims_to_go.victims
        rospy.Rate(10).sleep()
        victim = VictimInfoMsg()
        victim.id = 3
        victim.victimPose.pose.position.x = 1
        victim.victimPose.pose.position.y = 2.1
        victim.victimPose.pose.position.z = 5.2
        victim.victimPose.pose.orientation.x = 0.1
        victim.victimPose.pose.orientation.y = 0.5
        victim.victimPose.pose.orientation.z = 1
        victim.victimPose.pose.orientation.w = 0.9
        victim.probability = 0.5
        global_vars.test_agent.target_victim_ = victim
        cost_function = agent_cost_functions.update_victim_cost_function.\
            UpdateVictimCostFunction(global_vars.test_agent)
        cost = cost_function.execute()
        self.assertEqual(cost, 1)

    def test_validation_state_execute(self):
        rospy.loginfo('test_validation_state_execute')
        global_vars.test_agent.all_states_["validation_state"].execute()
        rospy.Rate(10).sleep()
        self.assertEqual(global_vars.test_agent.valid_victims_, 1)

    def test_validation_state_exploration(self):
        rospy.loginfo('test_validation_state_exploration')
        global_vars.test_agent.current_robot_state_ = robotModeMsg.MODE_DF_HOLD
        global_vars.test_agent.current_robot_pose_.pose.position.x = 4.8
        global_vars.test_agent.current_robot_pose_.pose.position.y = 11.6
        victims_to_go = WorldModelMsg()
        victim = VictimInfoMsg()
        victim.id = 8
        victim.victimPose.pose.position.x = 2
        victim.victimPose.pose.position.y = 6
        victim.victimPose.pose.position.z = 3.5
        victim.victimPose.pose.orientation.x = 0.1
        victim.victimPose.pose.orientation.y = 0.5
        victim.victimPose.pose.orientation.z = 1
        victim.victimPose.pose.orientation.w = 0.9
        victim.probability = 0.3
        victims_to_go.victims.append(victim)
        global_vars.test_agent.new_victims_ = victims_to_go.victims
        victim = VictimInfoMsg()
        victim.id = 8
        victim.victimPose.pose.position.x = 2
        victim.victimPose.pose.position.y = 6
        victim.victimPose.pose.position.z = 3.5
        victim.victimPose.pose.orientation.x = 0.1
        victim.victimPose.pose.orientation.y = 0.5
        victim.victimPose.pose.orientation.z = 1
        victim.victimPose.pose.orientation.w = 0.9
        victim.probability = 0.2
        global_vars.test_agent.target_victim_ = victim
        next_state = global_vars.test_agent.\
            all_states_["validation_state"].make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "scan_end_effector_planner_state")
        self.assertEqual(global_vars.test_agent.current_robot_state_,
                         robotModeMsg.MODE_EXPLORATION)

    def test_validation_state_find_new_victim(self):
        rospy.loginfo('test_validation_state_find_new_victim')
        global_vars.test_agent.current_robot_state_ = robotModeMsg.MODE_DF_HOLD
        global_vars.test_agent.current_robot_pose_.pose.position.x = 1.8
        global_vars.test_agent.current_robot_pose_.pose.position.y = 8.6
        victims_to_go = WorldModelMsg()
        victim = VictimInfoMsg()
        victim.id = 8
        victim.victimPose.pose.position.x = 2
        victim.victimPose.pose.position.y = 6
        victim.victimPose.pose.position.z = 3.5
        victim.victimPose.pose.orientation.x = 0.1
        victim.victimPose.pose.orientation.y = 0.5
        victim.victimPose.pose.orientation.z = 1
        victim.victimPose.pose.orientation.w = 0.9
        victim.probability = 0.3
        victims_to_go.victims.append(victim)
        victim = VictimInfoMsg()
        victim.id = 9
        victim.victimPose.pose.position.x = 2
        victim.victimPose.pose.position.y = 9
        victim.victimPose.pose.position.z = 3.2
        victim.victimPose.pose.orientation.x = 0.1
        victim.victimPose.pose.orientation.y = 0.5
        victim.victimPose.pose.orientation.z = 1
        victim.victimPose.pose.orientation.w = 0.9
        victim.probability = 0.2
        victims_to_go.victims.append(victim)
        global_vars.test_agent.new_victims_ = victims_to_go.victims
        victim = VictimInfoMsg()
        victim.id = 8
        victim.victimPose.pose.position.x = 2
        victim.victimPose.pose.position.y = 6
        victim.victimPose.pose.position.z = 3.5
        victim.victimPose.pose.orientation.x = 0.1
        victim.victimPose.pose.orientation.y = 0.5
        victim.victimPose.pose.orientation.z = 1
        victim.victimPose.pose.orientation.w = 0.9
        victim.probability = 0.2
        global_vars.test_agent.target_victim_ = victim
        next_state = global_vars.test_agent.\
            all_states_["validation_state"].make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "track_end_effector_planner_state")
        self.assertEqual(global_vars.test_agent.current_robot_state_,
                         robotModeMsg.MODE_IDENTIFICATION)

    def test_validation_state_stop_button(self):
        rospy.loginfo('test_validation_state_stop_button')
        global_vars.test_agent.current_robot_state_ = robotModeMsg.MODE_DF_HOLD
        global_vars.test_agent.transition_to_state(robotModeMsg.MODE_OFF)
        rospy.Rate(10).sleep()
        next_state = global_vars.test_agent.\
            all_states_["validation_state"].make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "stop_button_state")
        self.assertEqual(global_vars.test_agent.current_robot_state_,
                         robotModeMsg.MODE_OFF)

    def test_validation_state_teleoperation(self):
        rospy.loginfo('test_validation_state_teleoperation')
        global_vars.test_agent.current_robot_state_ = robotModeMsg.MODE_DF_HOLD
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
        victims_to_go = WorldModelMsg()
        victim = VictimInfoMsg()
        victim.id = 5
        victim.victimPose.pose.position.x = 3
        victim.victimPose.pose.position.y = 4
        victim.probability = 0.2
        victims_to_go.victims.append(victim)
        global_vars.com.victims_pub_.publish(victims_to_go)
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
        self.assertEqual(next_state, "test_and_park_end_effector_planner_state")
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

    def test_yellow_black_arena_exploration_strategy1_state_exploration(self):
        rospy.loginfo('test_yellow_black_arena_exploration_strategy1_state_\
                      exploration')
        global_vars.test_agent.current_robot_state_ = \
            robotModeMsg.MODE_EXPLORATION
        global_vars.test_agent.yellow_black_arena_area_explored_ = \
            global_vars.test_agent.max_yellow_black_area_ * 0.8
        next_state = global_vars.test_agent.\
            all_states_["yellow_black_arena_exploration_strategy1_state"].\
            make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(global_vars.test_agent.current_exploration_mode_,
                         DoExplorationGoal.TYPE_DEEP)

    def test_yellow_black_arena_exploration_strategy1_state_stop_button(self):
        rospy.loginfo('test_yellow_black_arena_exploration_strategy1_state_\
                      stop_button')
        global_vars.test_agent.current_robot_state_ = \
            robotModeMsg.MODE_EXPLORATION
        global_vars.test_agent.transition_to_state(robotModeMsg.MODE_OFF)
        rospy.Rate(10).sleep()
        next_state = global_vars.test_agent.\
            all_states_["yellow_black_arena_exploration_strategy1_state"].\
            make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "stop_button_state")
        self.assertEqual(global_vars.test_agent.current_robot_state_,
                         robotModeMsg.MODE_OFF)

    def test_yellow_black_arena_exploration_strategy1_state_teleoperation(self):
        rospy.loginfo('test_yellow_black_arena_exploration_strategy1_state_\
                      teleoperation')
        global_vars.test_agent.current_robot_state_ = \
            robotModeMsg.MODE_EXPLORATION
        global_vars.test_agent.transition_to_state(robotModeMsg.
                                                   MODE_TELEOPERATED_LOCOMOTION)
        rospy.Rate(10).sleep()
        next_state = global_vars.test_agent.\
            all_states_["yellow_black_arena_exploration_strategy1_state"].\
            make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "yellow_black_arena_teleoperation_state")
        self.assertEqual(global_vars.test_agent.current_robot_state_,
                         robotModeMsg.MODE_TELEOPERATED_LOCOMOTION)

    def test_yellow_black_arena_save_robot_pose_state_exploration(self):
        rospy.loginfo('test_yellow_black_arena_save_robot_pose_state_\
                      exploration')
        global_vars.test_agent.current_robot_state_ = \
            robotModeMsg.MODE_EXPLORATION
        starting_x = 1.8
        starting_y = 8.6
        global_vars.com.robot_pose_.pose.position.x = starting_x
        global_vars.com.robot_pose_.pose.position.y = starting_y
        next_state = global_vars.test_agent.\
            all_states_["yellow_black_arena_save_robot_pose_state"].execute()
        next_state = global_vars.test_agent.\
            all_states_["yellow_black_arena_save_robot_pose_state"].\
            make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state,
                         "yellow_black_arena_exploration_strategy1_state")
        self.assertGreater(global_vars.test_agent.
                           save_robot_pose_.pose.position.x, starting_x)
        self.assertGreater(global_vars.test_agent.
                           save_robot_pose_.pose.position.y, starting_y)
        self.assertEqual(global_vars.test_agent.current_exploration_mode_, -1)

    def test_yellow_black_arena_save_robot_pose_state_teleoperation(self):
        rospy.loginfo('test_yellow_black_arena_save_robot_pose_state_\
                      teleoperation')
        global_vars.test_agent.current_robot_state_ = \
            robotModeMsg.MODE_EXPLORATION
        global_vars.test_agent.transition_to_state(robotModeMsg.
                                                   MODE_TELEOPERATED_LOCOMOTION)
        rospy.Rate(10).sleep()
        next_state = global_vars.test_agent.\
            all_states_["yellow_black_arena_save_robot_pose_state"].\
            make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "teleoperation_state")
        self.assertEqual(global_vars.test_agent.current_robot_state_,
                         robotModeMsg.MODE_TELEOPERATED_LOCOMOTION)

    def test_yellow_black_arena_turn_back_check_state_aborted(self):
        rospy.loginfo('test_yellow_black_arena_turn_back_check_state_aborted')
        global_vars.test_agent.current_robot_state_ = \
            robotModeMsg.MODE_EXPLORATION
        next_state = global_vars.test_agent.\
            all_states_["yellow_black_arena_turn_back_move_base_state"].\
            execute()
        global_vars.com.move_base_aborted_ = True
        rospy.sleep(2.)
        next_state = global_vars.test_agent.\
            all_states_["yellow_black_arena_turn_back_check_state"].\
            make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state,
                         "yellow_black_arena_turn_back_move_base_state")

    def test_yellow_black_arena_turn_back_check_state_stop_button(self):
        rospy.loginfo('test_yellow_black_arena_turn_back_check_state_\
                      stop_button')
        global_vars.test_agent.current_robot_state_ = \
            robotModeMsg.MODE_EXPLORATION
        global_vars.test_agent.transition_to_state(robotModeMsg.MODE_OFF)
        rospy.Rate(10).sleep()
        next_state = global_vars.test_agent.\
            all_states_["yellow_black_arena_turn_back_check_state"].\
            make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "stop_button_state")
        self.assertEqual(global_vars.test_agent.current_robot_state_,
                         robotModeMsg.MODE_OFF)

    def test_yellow_black_arena_turn_back_check_state_succeeded(self):
        rospy.loginfo('test_yellow_black_arena_turn_back_check_state_succeeded')
        global_vars.test_agent.current_robot_state_ = \
            robotModeMsg.MODE_EXPLORATION
        next_state = global_vars.test_agent.\
            all_states_["yellow_black_arena_turn_back_move_base_state"].\
            execute()
        global_vars.com.move_base_succeeded_ = True
        rospy.sleep(2.)
        next_state = global_vars.test_agent.\
            all_states_["yellow_black_arena_turn_back_check_state"].\
            make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "teleoperation_state")
        self.assertEqual(global_vars.test_agent.current_robot_state_,
                         robotModeMsg.MODE_TELEOPERATED_LOCOMOTION)

    def test_yellow_black_arena_turn_back_check_state_teleoperation(self):
        rospy.loginfo('test_yellow_black_arena_turn_back_check_state_\
                      teleoperation')
        global_vars.test_agent.current_robot_state_ = \
            robotModeMsg.MODE_EXPLORATION
        global_vars.test_agent.transition_to_state(robotModeMsg.
                                                   MODE_TELEOPERATED_LOCOMOTION)
        rospy.Rate(10).sleep()
        next_state = global_vars.test_agent.\
            all_states_["yellow_black_arena_turn_back_check_state"].\
            make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "teleoperation_state")
        self.assertEqual(global_vars.test_agent.current_robot_state_,
                         robotModeMsg.MODE_TELEOPERATED_LOCOMOTION)

    def test_yellow_black_arena_turn_back_move_base_state_check(self):
        rospy.loginfo('test_yellow_black_arena_turn_back_move_base_state_check')
        global_vars.test_agent.current_robot_state_ = \
            robotModeMsg.MODE_EXPLORATION
        next_state = global_vars.test_agent.\
            all_states_["yellow_black_arena_turn_back_move_base_state"].\
            make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "yellow_black_arena_turn_back_check_state")

    def test_yellow_black_arena_turn_back_move_base_state_teleoperation(self):
        rospy.loginfo('test_yellow_black_arena_turn_back_move_base_state_\
                      teleoperation')
        global_vars.test_agent.current_robot_state_ = \
            robotModeMsg.MODE_EXPLORATION
        global_vars.test_agent.transition_to_state(robotModeMsg.
                                                   MODE_TELEOPERATED_LOCOMOTION)
        rospy.Rate(10).sleep()
        next_state = global_vars.test_agent.\
            all_states_["yellow_black_arena_turn_back_move_base_state"].\
            make_transition()
        rospy.Rate(10).sleep()
        self.assertEqual(next_state, "teleoperation_state")
        self.assertEqual(global_vars.test_agent.current_robot_state_,
                         robotModeMsg.MODE_TELEOPERATED_LOCOMOTION)

if __name__ == '__main__':
    rospy.init_node('test_agent')
    global_vars.init()
    rospy.sleep(3.)
    suite = unittest.TestLoader().loadTestsFromTestCase(TestAgent)
    unittest.TextTestRunner(verbosity=1).run(suite)
    global_vars.com.delete_action_servers()
    rospy.signal_shutdown('Unit tests finished')
