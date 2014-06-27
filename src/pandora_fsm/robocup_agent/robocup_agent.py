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
import actionlib
import threading
import state_manager
import dynamic_reconfigure.server
import agent
import agent_topics
import robocup_states
import robocup_cost_functions

from pandora_fsm.cfg import FSMParamsConfig
from math import sqrt, pow

from geometry_msgs.msg import PoseStamped
from state_manager_communications.msg import robotModeMsg
from std_msgs.msg import Int32, Float32
from pandora_end_effector_planner.msg import MoveEndEffectorAction
from pandora_rqt_gui.msg import ValidateVictimGUIAction
from pandora_data_fusion_msgs.msg import WorldModelMsg, VictimInfoMsg, \
    QrNotificationMsg, ValidateVictimAction, DeleteVictimAction
from move_base_msgs.msg import MoveBaseAction
from pandora_navigation_msgs.msg import ArenaTypeMsg, DoExplorationAction


class RoboCupAgent(agent.Agent, state_manager.state_client.StateClient):

    def __init__(self):

        state_manager.state_client.StateClient.__init__(self)

        self.current_arena_ = ArenaTypeMsg.TYPE_YELLOW
        self.yellow_arena_area_explored_ = 0
        self.yellow_black_arena_area_explored_ = 0
        self.current_score_ = 0
        self.valid_victims_ = 0
        self.qrs_ = 0
        self.current_robot_pose_ = PoseStamped()
        self.current_exploration_mode_ = -1

        self.aborted_victims_ = []
        self.new_victims_ = []
        self.target_victim_ = VictimInfoMsg()

        self.new_robot_state_cond_ = threading.Condition()
        self.current_robot_state_cond_ = threading.Condition()
        self.current_robot_state_ = robotModeMsg.MODE_OFF

        dynamic_reconfigure.server.Server(FSMParamsConfig, self.reconfigure)

        self.define_states()
        self.current_state_ = self.all_states_["waiting_to_start_state"]

        self.strategy3_fast_limit_ = self.strategy3_deep_limit_ * 1.4

        self.strategy4_previous_victims_ = 0
        self.strategy4_previous_qrs_ = 0
        self.strategy4_previous_area_ = 0
        self.strategy4_previous_resets_ = 0
        self.strategy4_previous_restarts_ = 0
        self.strategy4_current_cost_ = 0

        self.strategy5_fast_limit_ = self.strategy5_deep_limit_ * 1.4

        rospy.Subscriber(agent_topics.arena_type_topic, ArenaTypeMsg,
                         self.arena_type_cb)
        rospy.Subscriber(agent_topics.robocup_score_topic, Int32, self.score_cb)
        rospy.Subscriber(agent_topics.qr_notification_topic, QrNotificationMsg,
                         self.qr_notification_cb)
        rospy.Subscriber(agent_topics.area_covered_topic, Float32,
                         self.area_covered_cb)
        rospy.Subscriber(agent_topics.world_model_topic, WorldModelMsg,
                         self.world_model_cb)

        self.do_exploration_ac_ = \
            actionlib.SimpleActionClient(agent_topics.do_exploration_topic,
                                         DoExplorationAction)
        #~ self.do_exploration_ac_.wait_for_server()

        self.move_base_ac_ = \
            actionlib.SimpleActionClient(agent_topics.move_base_topic,
                                         MoveBaseAction)
        #~ self.move_base_ac_.wait_for_server()

        self.delete_victim_ac_ = \
            actionlib.SimpleActionClient(agent_topics.delete_victim_topic,
                                         DeleteVictimAction)
        #~ self.delete_victim_ac_.wait_for_server()

        self.gui_validate_victim_ac_ = \
            actionlib.SimpleActionClient(agent_topics.gui_validation_topic,
                                         ValidateVictimGUIAction)
        #~ self.gui_validate_victim_ac_.wait_for_server()

        self.data_fusion_validate_victim_ac_ = \
            actionlib.SimpleActionClient(agent_topics.
                                         data_fusion_validate_victim_topic,
                                         ValidateVictimAction)
        #~ self.data_fusion_validate_victim_ac_.wait_for_server()

        self.end_effector_planner_ac_ = \
            actionlib.SimpleActionClient(agent_topics.
                                         move_end_effector_planner_topic,
                                         MoveEndEffectorAction)
        #~ self.end_effector_planner_ac_.wait_for_server()

        self.client_initialize()

    def main(self):
        rospy.loginfo("Agent started")
        while not rospy.is_shutdown():
            self.current_state_.execute()
            new_state = self.current_state_.make_transition()
            if new_state != self.current_state_.name_:
                rospy.loginfo("Agent is transitioning from state %s to %s",
                              self.current_state_.name_, new_state)

            self.current_state_ = self.all_states_[new_state]
            rospy.Rate(10).sleep()

    def define_states(self):
        self.all_states_ = \
            {"waiting_to_start_state":
                robocup_states.WaitingToStartState(
                    self,
                    ["teleoperation_state",
                     "waiting_to_start_state",
                     "test_and_park_end_effector_planner_state"]
                ),
                "test_and_park_end_effector_planner_state":
                robocup_states.TestAndParkEndEffectorPlannerState(
                    self,
                    ["teleoperation_state",
                     "robot_start_state",
                     "waiting_to_start_state"]
                ),
                "robot_start_state":
                robocup_states.RobotStartState(
                    self,
                    ["teleoperation_state",
                     "stop_button_state",
                     "robot_start_state",
                     "scan_end_effector_planner_state"]
                ),
                "scan_end_effector_planner_state":
                robocup_states.ScanEndEffectorPlannerState(
                    self,
                    ["teleoperation_state",
                     self.exploration_strategy_]
                ),
                "exploration_strategy1_state":
                robocup_states.ExplorationStrategy1State(
                    self,
                    ["teleoperation_state",
                     "stop_button_state",
                     self.exploration_strategy_,
                     "track_end_effector_planner_state"],
                    [robocup_cost_functions.
                     FindNewVictimToGoCostFunction(self),
                     robocup_cost_functions.
                     ExplorationModeCostFunction(self)]
                ),
                "exploration_strategy2_state":
                robocup_states.ExplorationStrategy2State(
                    self,
                    ["teleoperation_state",
                     "stop_button_state",
                     self.exploration_strategy_,
                     "track_end_effector_planner_state"],
                    [robocup_cost_functions.
                     FindNewVictimToGoCostFunction(self),
                     robocup_cost_functions.
                     ExplorationModeCostFunction2(self)]
                ),
                "exploration_strategy3_state":
                robocup_states.ExplorationStrategy3State(
                    self,
                    ["teleoperation_state",
                     "stop_button_state",
                     self.exploration_strategy_,
                     "track_end_effector_planner_state"],
                    [robocup_cost_functions.
                     FindNewVictimToGoCostFunction(self),
                     robocup_cost_functions.
                     ExplorationModeCostFunction3(self)]
                ),
                "exploration_strategy4_state":
                robocup_states.ExplorationStrategy4State(
                    self,
                    ["teleoperation_state",
                     "stop_button_state",
                     self.exploration_strategy_,
                     "track_end_effector_planner_state"],
                    [robocup_cost_functions.
                     FindNewVictimToGoCostFunction(self),
                     robocup_cost_functions.
                     ExplorationModeCostFunction4(self)]
                ),
                "exploration_strategy5_state":
                robocup_states.ExplorationStrategy5State(
                    self,
                    ["teleoperation_state",
                     "stop_button_state",
                     self.exploration_strategy_,
                     "track_end_effector_planner_state"],
                    [robocup_cost_functions.
                     FindNewVictimToGoCostFunction(self),
                     robocup_cost_functions.
                     ExplorationModeCostFunction3(self)]
                ),
                "old_exploration_state":
                robocup_states.OldExplorationState(
                    self,
                    ["teleoperation_state",
                     "stop_button_state",
                     self.exploration_strategy_,
                     "track_end_effector_planner_state"],
                    [robocup_cost_functions.
                     FindNewVictimToGoCostFunction(self)]
                ),
                "track_end_effector_planner_state":
                robocup_states.TrackEndEffectorPlannerState(
                    self,
                    ["teleoperation_state",
                     "identification_move_to_victim_state"]
                ),
                "identification_move_to_victim_state":
                robocup_states.IdentificationMoveToVictimState(
                    self,
                    ["teleoperation_state",
                     "identification_check_for_victims_state"]
                ),
                "identification_check_for_victims_state":
                robocup_states.IdentificationCheckForVictimsState(
                    self,
                    ["teleoperation_state",
                     "stop_button_state",
                     "identification_check_for_victims_state",
                     "track_end_effector_planner_state",
                     "data_fusion_hold_state",
                     "scan_end_effector_planner_state"],
                    [robocup_cost_functions.
                     FindNewVictimToGoCostFunction(self),
                     robocup_cost_functions.
                     UpdateVictimCostFunction(self)]
                ),
                "data_fusion_hold_state":
                robocup_states.DataFusionHoldState(
                    self,
                    ["teleoperation_state",
                     "stop_button_state",
                     "data_fusion_hold_state",
                     "validation_state",
                     "track_end_effector_planner_state",
                     "scan_end_effector_planner_state"],
                    [robocup_cost_functions.
                     FindNewVictimToGoCostFunction(self)]
                ),
                "validation_state":
                robocup_states.ValidationState(
                    self,
                    ["teleoperation_state",
                     "stop_button_state",
                     "track_end_effector_planner_state",
                     "scan_end_effector_planner_state"],
                    [robocup_cost_functions.
                     FindNewVictimToGoCostFunction(self)]
                ),
                "yellow_black_arena_save_robot_pose_state":
                robocup_states.YellowBlackArenaSaveRobotPoseState(
                    self,
                    ["teleoperation_state",
                     "yellow_black_arena_exploration_strategy1_state"]
                ),
                "yellow_black_arena_exploration_strategy1_state":
                robocup_states.YellowBlackArenaExplorationStrategy1State(
                    self,
                    ["teleoperation_state",
                     "stop_button_state",
                     "yellow_black_arena_exploration_strategy1_state"]
                ),
                "yellow_black_arena_turn_back_move_base_state":
                robocup_states.YellowBlackArenaTurnBackMoveBaseState(
                    self,
                    ["teleoperation_state",
                     "yellow_black_arena_turn_back_check_state"]
                ),
                "yellow_black_arena_turn_back_check_state":
                robocup_states.YellowBlackArenaTurnBackCheckState(
                    self,
                    ["teleoperation_state",
                     "stop_button_state",
                     "yellow_black_arena_turn_back_check_state",
                     "yellow_black_arena_turn_back_move_base_state"]
                ),
                "teleoperation_state":
                robocup_states.TeleoperationState(
                    self,
                    ["teleoperation_state",
                     "stop_button_state",
                     "scan_end_effector_planner_state"]
                ),
                "stop_button_state":
                robocup_states.StopButtonState(
                    self,
                    ["teleoperation_state",
                     "stop_button_state",
                     "scan_end_effector_planner_state"]
                )}

    def calculate_distance_2d(self, object1, object2):
        dist = sqrt(pow(object1.x - object2.x, 2) +
                    pow(object1.y - object2.y, 2))
        return dist

    def calculate_distance_3d(self, object1, object2):
        dist = sqrt(pow(object1.x - object2.x, 2) +
                    pow(object1.y - object2.y, 2) +
                    pow(object1.z - object2.z, 2))
        return dist

    def start_transition(self, state):
        rospy.loginfo("[%s] Starting Transition to state %i", self._name, state)
        self.new_robot_state_cond_.acquire()
        self.current_robot_state_ = state
        self.new_robot_state_cond_.notify()
        self.new_robot_state_cond_.wait()
        self.new_robot_state_cond_.release()
        self.transition_complete(state)

    def complete_transition(self):
        rospy.loginfo("[%s] System Transitioned, starting work", self._name)
        self.current_robot_state_cond_.acquire()
        self.current_robot_state_cond_.notify()
        self.current_robot_state_cond_.release()

    def arena_type_cb(self, msg):
        if self.current_arena_ == msg.TYPE_YELLOW:
            self.current_arena_ = msg.arena_type

    def qr_notification_cb(self, msg):
        self.qrs_ += 1

    def score_cb(self, msg):
        self.current_score_ = msg.data

    def area_covered_cb(self, msg):
        if self.current_state_.name_ != \
                "yellow_black_arena_save_robot_pose_state":
            self.yellow_arena_area_explored_ = msg.data
        else:
            self.yellow_black_arena_area_explored_ = \
                msg.data - self.yellow_arena_area_explored_

    def world_model_cb(self, msg):
        self.new_victims_ = msg.victims

    def reconfigure(self, config, level):
        self.max_time_ = config["maxTime"]
        self.max_victims_ = config["arenaVictims"]
        self.max_qrs_ = config["maxQRs"]
        self.max_yellow_area_ = config["yellowArenaArea"]
        self.max_yellow_black_area_ = config["yellowBlackArenaArea"]
        self.initial_time_ = rospy.get_rostime().secs - config["timePassed"]
        self.minutes_passed_ = config["timePassed"]
        self.valid_victim_probability_ = config["validVictimProbability"]
        self.aborted_victims_distance_ = config["abortedVictimsDistance"]
        self.robot_resets_ = config["robotResets"]
        self.robot_restarts_ = config["robotRestarts"]
        if config["explorationStrategy"] == 0:
            self.exploration_strategy_ = "old_exploration_state"
        elif config["explorationStrategy"] == 1:
            self.exploration_strategy_ = "exploration_strategy1_state"
        elif config["explorationStrategy"] == 2:
            self.exploration_strategy_ = "exploration_strategy2_state"
        elif config["explorationStrategy"] == 3:
            self.exploration_strategy_ = "exploration_strategy3_state"
        elif config["explorationStrategy"] == 4:
            self.exploration_strategy_ = "exploration_strategy4_state"
        elif config["explorationStrategy"] == 5:
            self.exploration_strategy_ = "exploration_strategy5_state"
        elif config["explorationStrategy"] == 6:
            self.exploration_strategy_ = \
                "yellow_black_arena_save_robot_pose_state"
        elif config["explorationStrategy"] == 7:
            self.exploration_strategy_ = "yellow_black_arena_turn_back_state"
        self.strategy3_deep_limit_ = config["strategy3DeepLimit"]
        self.strategy4_deep_limit_ = config["strategy4DeepLimit"]
        self.strategy4_fast_limit_ = config["strategy4FastLimit"]
        self.strategy5_deep_limit_ = config["strategy5DeepLimit"]
        self.define_states()
        return config


def main():

    rospy.init_node('agent')

    agent = RoboCupAgent()

    agent.main()

    rospy.loginfo('agent terminated')

if __name__ == '__main__':
    main()
