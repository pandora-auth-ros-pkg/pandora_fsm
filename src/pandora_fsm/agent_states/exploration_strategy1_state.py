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

from state_manager_communications.msg import robotModeMsg
from pandora_navigation_msgs.msg import ArenaTypeMsg, DoExplorationGoal
from pandora_end_effector_planner.msg import MoveEndEffectorGoal


class ExplorationStrategy1State(state.State):

    def __init__(self, agent, next_states, cost_functions=None):
        state.State.__init__(self, agent, next_states, cost_functions)
        self.name_ = "exploration_strategy1_state"

    def execute(self):
        pass

    def make_transition(self):
        if self.agent_.current_robot_state_ == \
                robotModeMsg.MODE_TELEOPERATED_LOCOMOTION:
            self.agent_.end_exploration()
            self.agent_.preempt_end_effector_planner()
            self.agent_.park_end_effector_planner()
            self.agent_.new_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.notify()
            self.agent_.current_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.release()
            self.agent_.current_robot_state_cond_.wait()
            self.agent_.current_robot_state_cond_.release()
            return self.next_states_[0]
        elif self.agent_.current_robot_state_ == robotModeMsg.MODE_OFF:
            self.agent_.end_exploration()
            self.agent_.preempt_end_effector_planner()
            self.agent_.park_end_effector_planner()
            self.agent_.new_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.notify()
            self.agent_.current_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.release()
            self.agent_.current_robot_state_cond_.wait()
            self.agent_.current_robot_state_cond_.release()
            return self.next_states_[1]

        new_victims_cost = self.cost_functions_[0].execute()
        max_victim_cost = 0
        for i in range(0, len(new_victims_cost)):
            if new_victims_cost[i] > max_victim_cost:
                max_victim_cost = new_victims_cost[i]
                max_victim = self.agent_.new_victims_[i]

        if max_victim_cost > 0:
            self.agent_.end_exploration()
            self.agent_.target_victim_ = max_victim
            self.agent_.new_robot_state_cond_.acquire()
            self.agent_.transition_to_state(robotModeMsg.MODE_IDENTIFICATION)
            self.agent_.new_robot_state_cond_.wait()
            self.agent_.new_robot_state_cond_.notify()
            self.agent_.current_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.release()
            self.agent_.current_robot_state_cond_.wait()
            self.agent_.current_robot_state_cond_.release()
            return self.next_states_[3]

        current_cost = self.cost_functions_[1].execute()

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
                self.agent_.end_exploration()
                self.agent_.preempt_end_effector_planner()
                self.agent_.park_end_effector_planner()
                self.agent_.new_robot_state_cond_.acquire()
                self.agent_.\
                    transition_to_state(robotModeMsg.
                                        MODE_TELEOPERATED_LOCOMOTION)
                self.agent_.new_robot_state_cond_.wait()
                self.agent_.new_robot_state_cond_.notify()
                self.agent_.current_robot_state_cond_.acquire()
                self.agent_.new_robot_state_cond_.release()
                self.agent_.current_robot_state_cond_.wait()
                self.agent_.current_robot_state_cond_.release()
                return self.next_states_[0]

        return self.next_states_[2]

    def start_exploration(self, exploration_mode):
        if self.agent_.current_exploration_mode_ != -1:
            self.agent_.end_exploration()

        rospy.sleep(1.)
        self.agent_.current_exploration_mode_ = exploration_mode
        goal = DoExplorationGoal(exploration_type=exploration_mode)
        self.agent_.do_exploration_ac_.send_goal(goal,
                                                 feedback_cb=self.feedback_cb,
                                                 done_cb=self.done_cb)

    def feedback_cb(self, feedback):
        self.agent_.current_robot_pose_ = feedback.base_position

    def done_cb(self, status, result):
        rospy.loginfo("navigation sent aborted")
        rospy.loginfo(self.agent_.do_exploration_ac_.get_goal_status_text())
        self.agent_.current_exploration_mode_ = -1
