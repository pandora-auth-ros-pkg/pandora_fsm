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
import state

from actionlib import GoalStatus

from state_manager_communications.msg import robotModeMsg
from pandora_data_fusion_msgs.msg import DeleteVictimGoal
from pandora_end_effector_planner.msg import MoveEndEffectorGoal


class IdentificationCheckForVictimsState(state.State):

    def __init__(self, agent, next_states, cost_functions=None):
        state.State.__init__(self, agent, next_states, cost_functions)
        self.name_ = "identification_check_for_victims_state"

    def execute(self):
        pass

    def make_transition(self):
        if self.agent_.current_robot_state_ == \
                robotModeMsg.MODE_TELEOPERATED_LOCOMOTION:
            self.agent_.move_base_ac_.cancel_all_goals()
            self.agent_.move_base_ac_.wait_for_result()
            self.agent_.end_effector_planner_ac_.cancel_all_goals()
            self.agent_.end_effector_planner_ac_.wait_for_result()
            goal = MoveEndEffectorGoal(command=MoveEndEffectorGoal.PARK)
            self.agent_.end_effector_planner_ac_.send_goal(goal)
            self.agent_.end_effector_planner_ac_.wait_for_result()
            self.agent_.new_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.notify()
            self.agent_.current_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.release()
            self.agent_.current_robot_state_cond_.wait()
            self.agent_.current_robot_state_cond_.release()
            return self.next_states_[0]
        elif self.agent_.current_robot_state_ == robotModeMsg.MODE_OFF:
            self.agent_.move_base_ac_.cancel_all_goals()
            self.agent_.move_base_ac_.wait_for_result()
            self.agent_.end_effector_planner_ac_.cancel_all_goals()
            self.agent_.end_effector_planner_ac_.wait_for_result()
            goal = MoveEndEffectorGoal(command=MoveEndEffectorGoal.PARK)
            self.agent_.end_effector_planner_ac_.send_goal(goal)
            self.agent_.end_effector_planner_ac_.wait_for_result()
            self.agent_.new_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.notify()
            self.agent_.current_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.release()
            self.agent_.current_robot_state_cond_.wait()
            self.agent_.current_robot_state_cond_.release()
            return self.next_states_[1]

        if self.agent_.end_effector_planner_ac_.get_state() == \
                GoalStatus.ABORTED:
            self.agent_.move_base_ac_.cancel_all_goals()
            self.agent_.move_base_ac_.wait_for_result()
            new_victims_cost = self.cost_functions_[0].execute()
            max_victim_cost = 0
            for i in range(0, len(new_victims_cost)):
                if new_victims_cost[i] > max_victim_cost:
                    max_victim_cost = new_victims_cost[i]
                    max_victim = self.agent_.new_victims_[i]

            if max_victim_cost > 0:
                self.agent_.move_base_ac_.cancel_all_goals()
                self.agent_.move_base_ac_.wait_for_result()
                self.agent_.target_victim_ = max_victim
                return self.next_states_[3]

            self.agent_.new_robot_state_cond_.acquire()
            self.agent_.transition_to_state(robotModeMsg.MODE_EXPLORATION)
            self.agent_.new_robot_state_cond_.wait()
            self.agent_.new_robot_state_cond_.notify()
            self.agent_.current_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.release()
            self.agent_.current_robot_state_cond_.wait()
            self.agent_.current_robot_state_cond_.release()
            return self.next_states_[5]

        updated_victim = self.cost_functions_[1].execute()
        if updated_victim == 1:
            self.agent_.move_base_ac_.cancel_all_goals()
            self.agent_.move_base_ac_.wait_for_result()
            return self.next_states_[3]

        if self.agent_.move_base_ac_.get_state() == GoalStatus.SUCCEEDED:
            return self.next_states_[4]
        elif self.agent_.move_base_ac_.get_state() == GoalStatus.ABORTED:
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
                self.agent_.move_base_ac_.wait_for_result()
                self.agent_.target_victim_ = max_victim
                return self.next_states_[3]

            self.agent_.end_effector_planner_ac_.cancel_all_goals()
            self.agent_.end_effector_planner_ac_.wait_for_result()
            goal = MoveEndEffectorGoal(command=MoveEndEffectorGoal.PARK)
            self.agent_.end_effector_planner_ac_.send_goal(goal)
            self.agent_.end_effector_planner_ac_.wait_for_result()
            self.agent_.new_robot_state_cond_.acquire()
            self.agent_.transition_to_state(robotModeMsg.MODE_EXPLORATION)
            self.agent_.new_robot_state_cond_.wait()
            self.agent_.new_robot_state_cond_.notify()
            self.agent_.current_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.release()
            self.agent_.current_robot_state_cond_.wait()
            self.agent_.current_robot_state_cond_.release()
            return self.next_states_[5]

        return self.next_states_[2]
