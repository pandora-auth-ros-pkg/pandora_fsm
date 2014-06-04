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
import state

from state_manager_communications.msg import robotModeMsg
from pandora_rqt_gui.msg import ValidateVictimGUIGoal
from pandora_data_fusion_msgs.msg import ValidateVictimGoal, DeleteVictimGoal
from move_base_msgs.msg import MoveBaseGoal
from pandora_navigation_msgs.msg import ArenaTypeMsg, DoExplorationGoal


class WaitingToStartState(state.State):

    def execute(self):
        rospy.loginfo('WaitingToStartState execute')

    def make_transition(self):
        rospy.loginfo('WaitingToStartState make_transition')
        if self.agent_.new_robot_state_ == \
                robotModeMsg.MODE_TELEOPERATED_LOCOMOTION:
            self.agent_.new_robot_state_ack_ = self.agent_.new_robot_state_
            while not rospy.is_shutdown() and \
                    self.agent_.current_robot_state_ != \
                    self.agent_.new_robot_state_ack_:
                pass
            return self.next_states_[0]
        elif self.agent_.new_robot_state_ == robotModeMsg.MODE_START_AUTONOMOUS:
            self.agent_.new_robot_state_ack_ = self.agent_.new_robot_state_
            while not rospy.is_shutdown() and \
                    self.agent_.current_robot_state_ != \
                    self.agent_.new_robot_state_ack_:
                pass
            return self.next_states_[2]
        else:
            return self.next_states_[1]


class RobotStartState(state.State):

    def __init__(self, agent, next_states, cost_functions=None):
        state.State.__init__(self, agent, next_states, cost_functions)
        self.counter_ = 0

    def execute(self):
        rospy.loginfo('RobotStartState execute')
        self.wait_for_slam()

    def make_transition(self):
        rospy.loginfo('RobotStartState make_transition')
        if self.agent_.new_robot_state_ == \
                robotModeMsg.MODE_TELEOPERATED_LOCOMOTION:
            self.counter_ = 0
            self.agent_.new_robot_state_ack_ = self.agent_.new_robot_state_
            while not rospy.is_shutdown() and \
                    self.agent_.current_robot_state_ != \
                    self.agent_.new_robot_state_ack_:
                pass
            return self.next_states_[0]

        if self.counter_ == 10:
            self.counter_ = 0
            self.agent_.transition_to_state(robotModeMsg.MODE_EXPLORATION)
            self.agent_.new_robot_state_ack_ = robotModeMsg.MODE_EXPLORATION
            while not rospy.is_shutdown() and \
                    self.agent_.current_robot_state_ != \
                    self.agent_.new_robot_state_ack_:
                pass
            return self.next_states_[2]
        return self.next_states_[1]

    def wait_for_slam(self):
        rospy.loginfo('wait_for_slam')
        self.counter_ += 1
        rospy.sleep(1.)


class ExplorationState(state.State):

    def execute(self):
        rospy.loginfo('ExplorationState execute')

    def make_transition(self):
        rospy.loginfo('ExplorationState make_transition')
        if self.agent_.new_robot_state_ == \
                robotModeMsg.MODE_TELEOPERATED_LOCOMOTION:
            self.agent_.new_robot_state_ack_ = self.agent_.new_robot_state_
            while not rospy.is_shutdown() and \
                    self.agent_.current_robot_state_ != \
                    self.agent_.new_robot_state_ack_:
                pass
            return self.next_states_[0]

        new_victims_cost = self.cost_functions_[0].execute()
        max_victim_cost = 0
        for i in range(0, len(new_victims_cost)):
            if new_victims_cost[i] > max_victim_cost:
                max_victim_cost = new_victims_cost[i]
                max_victim = self.agent_.new_victims_[i]

        if max_victim_cost > 0:
            self.end_exploration()
            self.agent_.target_victim_ = max_victim
            self.agent_.transition_to_state(robotModeMsg.MODE_IDENTIFICATION)
            self.agent_.new_robot_state_ack_ = robotModeMsg.MODE_IDENTIFICATION
            while not rospy.is_shutdown() and \
                    self.agent_.current_robot_state_ != \
                    self.agent_.new_robot_state_ack_:
                pass
            return self.next_states_[2]

        current_cost = 0
        for function in self.cost_functions_[1]:
            current_cost += function.execute()

        current_cost /= len(self.cost_functions_)

        if self.agent_.current_arena_ == ArenaTypeMsg.TYPE_YELLOW:
            if current_cost < 25:
                if self.agent_.current_exploration_mode_ != \
                        DoExplorationGoal.TYPE_DEEP:
                    self.start_exploration(DoExplorationGoal.TYPE_DEEP)
            elif current_cost < 35:
                if self.agent_.current_exploration_mode_ != \
                        DoExplorationGoal.TYPE_NORMAL:
                    self.start_exploration(DoExplorationGoal.TYPE_NORMAL)
            else:
                if self.agent_.current_exploration_mode_ != \
                        DoExplorationGoal.TYPE_FAST:
                    self.start_exploration(DoExplorationGoal.TYPE_FAST)
        elif self.agent_.current_arena_ == ArenaTypeMsg.TYPE_ORANGE:
            if self.agent_.valid_victims_ == 0:
                if self.agent_.current_exploration_mode_ != \
                        DoExplorationGoal.TYPE_FAST:
                    self.start_exploration(DoExplorationGoal.TYPE_FAST)
            else:
                self.agent_.\
                    transition_to_state(robotModeMsg.
                                        MODE_TELEOPERATED_LOCOMOTION)
                self.agent_.new_robot_state_ack_ = \
                    robotModeMsg.MODE_TELEOPERATED_LOCOMOTION
                return self.next_states_[0]

        return self.next_states_[1]

    def start_exploration(self, exploration_mode):
        rospy.loginfo('start_exploration = %i' % exploration_mode)

        if self.agent_.current_exploration_mode_ != 0:
            self.end_exploration()

        rospy.Rate(2).sleep()
        self.agent_.current_exploration_mode_ = exploration_mode
        goal = DoExplorationGoal(exploration_type=exploration_mode)
        self.agent_.do_exploration_ac_.send_goal(goal,
                                                 feedback_cb=self.feedback_cb,
                                                 done_cb=self.done_cb)

    def end_exploration(self):
        rospy.loginfo('end_exploration')

        self.agent_.current_exploration_mode_ = 0
        self.agent_.do_exploration_ac_.cancel_all_goals()

    def feedback_cb(self, feedback):
        self.agent_.current_robot_pose_ = feedback.base_position

    def done_cb(self, status, result):
        self.agent_.current_exploration_mode_ = 0


class OldExplorationState(state.State):

    def execute(self):
        rospy.loginfo('OldExplorationState execute')

    def make_transition(self):
        rospy.loginfo('OldExplorationState make_transition')
        if self.agent_.new_robot_state_ == \
                robotModeMsg.MODE_TELEOPERATED_LOCOMOTION:
            self.agent_.new_robot_state_ack_ = self.agent_.new_robot_state_
            while not rospy.is_shutdown() and \
                    self.agent_.current_robot_state_ != \
                    self.agent_.new_robot_state_ack_:
                pass
            return self.next_states_[0]

        new_victims_cost = self.cost_functions_[0].execute()
        max_victim_cost = 0
        for i in range(0, len(new_victims_cost)):
            if new_victims_cost[i] > max_victim_cost:
                max_victim_cost = new_victims_cost[i]
                max_victim = self.agent_.new_victims_[i]

        if max_victim_cost > 0:
            self.end_exploration()
            self.agent_.target_victim_ = max_victim
            self.agent_.transition_to_state(robotModeMsg.MODE_IDENTIFICATION)
            self.agent_.new_robot_state_ack_ = robotModeMsg.MODE_IDENTIFICATION
            while not rospy.is_shutdown() and \
                    self.agent_.current_robot_state_ != \
                    self.agent_.new_robot_state_ack_:
                pass
            return self.next_states_[2]

        if self.agent_.current_exploration_mode_ != DoExplorationGoal.TYPE_DEEP:
            self.start_exploration(DoExplorationGoal.TYPE_DEEP)

        return self.next_states_[1]

    def start_exploration(self, exploration_mode):
        rospy.loginfo('start_exploration = %i' % exploration_mode)

        if self.agent_.current_exploration_mode_ != 0:
            self.end_exploration()

        rospy.Rate(2).sleep()
        self.agent_.current_exploration_mode_ = exploration_mode
        goal = DoExplorationGoal(explorationMode=exploration_mode)
        self.agent_.do_exploration_ac_.send_goal(goal,
                                                 feedback_cb=self.feedback_cb,
                                                 done_cb=self.done_cb)

    def end_exploration(self):
        rospy.loginfo('end_exploration')

        self.agent_.current_exploration_mode_ = 0
        self.agent_.do_exploration_ac_.cancel_all_goals()

    def feedback_cb(self, feedback):
        self.agent_.current_robot_pose_ = feedback.base_position

    def done_cb(self, status, result):
        self.agent_.current_exploration_mode_ = 0


class IdentificationState(state.State):

    def __init__(self, agent, next_states, cost_functions=None):
        state.State.__init__(self, agent, next_states, cost_functions)
        self.move_base_to_victim_ = True

    def execute(self):
        rospy.loginfo('IdentificationState execute')
        self.move_to_victim()

    def make_transition(self):
        rospy.loginfo('IdentificationState make_transition')
        if self.agent_.new_robot_state_ == \
                robotModeMsg.MODE_TELEOPERATED_LOCOMOTION:
            self.agent_.new_robot_state_ack_ = self.agent_.new_robot_state_
            while not rospy.is_shutdown() and \
                    self.agent_.current_robot_state_ != \
                    self.agent_.new_robot_state_ack_:
                pass
            return self.next_states_[0]

        updated_victim = self.cost_functions_[1].execute()
        if updated_victim == 1:
            self.agent_.move_base_ac_.cancel_all_goals()
            self.move_base_to_victim_ = True
            return self.next_states_[1]

        if self.agent_.move_base_ac_.get_state() == \
                actionlib.GoalStatus.SUCCEEDED:
            self.move_base_to_victim_ = True
            self.agent_.transition_to_state(robotModeMsg.MODE_DF_HOLD)
            self.agent_.new_robot_state_ack_ = robotModeMsg.MODE_DF_HOLD
            while not rospy.is_shutdown() and \
                    self.agent_.current_robot_state_ != \
                    self.agent_.new_robot_state_ack_:
                pass
            return self.next_states_[2]
        elif self.agent_.move_base_ac_.get_state() == \
                actionlib.GoalStatus.ABORTED:
            self.move_base_to_victim_ = True
            goal = DeleteVictimGoal(victimId=self.agent_.target_victim_.id)
            self.agent_.delete_victim_ac_.send_goal(goal)
            self.agent_.delete_victim_ac_.wait_for_result()

            for aborted_victim in self.agent_.aborted_victims_:
                if self.agent_.\
                        calculate_distance_3d(aborted_victim[0].victimPose.
                                              pose.position,
                                              self.agent_.target_victim_.
                                              victimPose.pose.position) < \
                        self.agent_.aborted_victims_distance_:
                    aborted_victim[1] += 1
                    break
            else:
                self.agent_.aborted_victims_.\
                    append([self.agent_.target_victim_, 1])

            rospy.sleep(1.)
            new_victims_cost = self.cost_functions_[0].execute()
            max_victim_cost = 0
            for i in range(0, len(new_victims_cost)):
                if new_victims_cost[i] > max_victim_cost:
                    max_victim_cost = new_victims_cost[i]
                    max_victim = self.agent_.new_victims_[i]

            if max_victim_cost > 0:
                self.agent_.move_base_ac_.cancel_all_goals()
                self.agent_.target_victim_ = max_victim
                return self.next_states_[1]

            self.agent_.transition_to_state(robotModeMsg.MODE_EXPLORATION)
            self.agent_.new_robot_state_ack_ = robotModeMsg.MODE_EXPLORATION
            while not rospy.is_shutdown() and \
                    self.agent_.current_robot_state_ != \
                    self.agent_.new_robot_state_ack_:
                pass
            return self.next_states_[3]

        return self.next_states_[1]

    def move_to_victim(self):
        rospy.loginfo('move_to_victim')

        if self.move_base_to_victim_:
            self.move_base_to_victim_ = False
            goal = MoveBaseGoal(target_pose=self.agent_.
                                target_victim_.victimPose)
            self.agent_.move_base_ac_.send_goal(goal,
                                                feedback_cb=self.feedback_cb)

    def feedback_cb(self, feedback):
        self.agent_.current_robot_pose_ = feedback.base_position


class DataFusionHoldState(state.State):

    def __init__(self, agent, next_states, cost_functions=None):
        state.State.__init__(self, agent, next_states, cost_functions)
        self.counter_ = 0

    def execute(self):
        rospy.loginfo('DataFusionHoldState execute')
        self.data_fusion_hold()

    def make_transition(self):
        rospy.loginfo('DataFusionHoldState make_transition')
        if self.agent_.new_robot_state_ == \
                robotModeMsg.MODE_TELEOPERATED_LOCOMOTION:
            self.counter_ = 0
            self.agent_.new_robot_state_ack_ = self.agent_.new_robot_state_
            while not rospy.is_shutdown() and \
                    self.agent_.current_robot_state_ != \
                    self.agent_.new_robot_state_ack_:
                pass
            return self.next_states_[0]

        if self.counter_ == 10:
            self.counter_ = 0
            for victim in self.agent_.new_victims_:
                if victim.id == self.agent_.target_victim_.id:
                    if victim.probability > \
                            self.agent_.valid_victim_probability_:
                        return self.next_states_[2]
                    else:
                        goal = ValidateVictimGoal()
                        goal.victimId = self.agent_.target_victim_.id
                        goal.victimValid = False
                        self.agent_.data_fusion_validate_victim_ac_.\
                            send_goal(goal)
                        self.agent_.data_fusion_validate_victim_ac_.\
                            wait_for_result()
                        rospy.sleep(1.)

                        new_victims_cost = self.cost_functions_[0].execute()
                        max_victim_cost = 0
                        for i in range(0, len(new_victims_cost)):
                            if new_victims_cost[i] > max_victim_cost:
                                max_victim_cost = new_victims_cost[i]
                                max_victim = self.agent_.new_victims_[i]

                        if max_victim_cost > 0:
                            self.agent_.target_victim_ = max_victim
                            self.agent_.transition_to_state(robotModeMsg.
                                                            MODE_IDENTIFICATION)
                            self.agent_.new_robot_state_ack_ = \
                                robotModeMsg.MODE_IDENTIFICATION
                            while not rospy.is_shutdown() and \
                                    self.agent_.current_robot_state_ != \
                                    self.agent_.new_robot_state_ack_:
                                pass
                            return self.next_states_[3]
                        self.agent_.transition_to_state(robotModeMsg.
                                                        MODE_EXPLORATION)
                        self.agent_.new_robot_state_ack_ = \
                            robotModeMsg.MODE_EXPLORATION
                        while not rospy.is_shutdown() and \
                                self.agent_.current_robot_state_ != \
                                self.agent_.new_robot_state_ack_:
                            pass
                        return self.next_states_[4]
        return self.next_states_[1]

    def data_fusion_hold(self):
        rospy.loginfo('data_fusion_hold')
        self.counter_ += 1
        rospy.sleep(1.)


class ValidationState(state.State):

    def execute(self):
        rospy.loginfo('ValidationState execute')
        self.reset_arena_type_to_yellow()
        self.validate_current_victim()

    def make_transition(self):
        rospy.loginfo('ValidationState make_transition')
        if self.agent_.new_robot_state_ == \
                robotModeMsg.MODE_TELEOPERATED_LOCOMOTION:
            self.agent_.new_robot_state_ack_ = self.agent_.new_robot_state_
            while not rospy.is_shutdown() and \
                    self.agent_.current_robot_state_ != \
                    self.agent_.new_robot_state_ack_:
                pass
            return self.next_states_[0]

        rospy.sleep(1.)

        new_victims_cost = self.cost_functions_[0].execute()
        max_victim_cost = 0
        for i in range(0, len(new_victims_cost)):
            if new_victims_cost[i] > max_victim_cost:
                max_victim_cost = new_victims_cost[i]
                max_victim = self.agent_.new_victims_[i]

        if max_victim_cost > 0:
            self.agent_.target_victim_ = max_victim
            self.agent_.transition_to_state(robotModeMsg.MODE_IDENTIFICATION)
            self.agent_.new_robot_state_ack_ = robotModeMsg.MODE_IDENTIFICATION
            while not rospy.is_shutdown() and \
                    self.agent_.current_robot_state_ != \
                    self.agent_.new_robot_state_ack_:
                pass
            return self.next_states_[1]
        self.agent_.transition_to_state(robotModeMsg.MODE_EXPLORATION)
        self.agent_.new_robot_state_ack_ = robotModeMsg.MODE_EXPLORATION
        while not rospy.is_shutdown() and \
                self.agent_.current_robot_state_ != \
                self.agent_.new_robot_state_ack_:
            pass
        return self.next_states_[2]

    def reset_arena_type_to_yellow(self):
        rospy.loginfo('reset_arena_type_to_yellow')
        self.agent_.current_arena_ = ArenaTypeMsg.TYPE_YELLOW

    def validate_current_victim(self):
        rospy.loginfo('validate_current_victim')
        goal = ValidateVictimGUIGoal()
        goal.victimFoundx = self.agent_.target_victim_.victimPose.pose.position.x
        goal.victimFoundy = self.agent_.target_victim_.victimPose.pose.position.y
        goal.probability = self.agent_.target_victim_.probability
        goal.sensorIDsFound = self.agent_.target_victim_.sensors
        self.agent_.gui_validate_victim_ac_.send_goal(goal)
        self.agent_.gui_validate_victim_ac_.wait_for_result()

        result = self.agent_.gui_validate_victim_ac_.get_result()

        if result.victimValid:
            self.agent_.valid_victims_ += 1

        goal = ValidateVictimGoal()
        goal.victimId = self.agent_.target_victim_.id
        goal.victimValid = result.victimValid
        self.agent_.data_fusion_validate_victim_ac_.send_goal(goal)
        self.agent_.data_fusion_validate_victim_ac_.wait_for_result()


class TeleoperationState(state.State):

    def execute(self):
        rospy.loginfo('TeleoperationState execute')
        pass

    def make_transition(self):
        rospy.loginfo('TeleoperationState make_transition')
        if self.agent_.new_robot_state_ == robotModeMsg.MODE_START_AUTONOMOUS:
            for victim in self.agent_.new_victims_:
                goal = DeleteVictimGoal(victimId=victim_.id)
                self.agent_.delete_victim_ac_.send_goal(goal)
                self.agent_.delete_victim_ac_.wait_for_result()

            self.agent_.new_robot_state_ack_ = self.agent_.new_robot_state_
            while not rospy.is_shutdown() and \
                    self.agent_.current_robot_state_ != \
                    self.agent_.new_robot_state_ack_:
                pass
            return self.next_states_[1]
        return self.next_states_[0]
