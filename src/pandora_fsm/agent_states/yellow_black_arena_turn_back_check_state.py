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

from sys import exit
from actionlib import GoalStatus

from state_manager_communications.msg import robotModeMsg
from pandora_end_effector_planner.msg import MoveEndEffectorGoal


class YellowBlackArenaTurnBackCheckState(state.State):

    def __init__(self, agent, next_states, cost_functions=None):
        state.State.__init__(self, agent, next_states, cost_functions)
        self.name_ = "yellow_black_arena_turn_back_check_state"

    def execute(self):
        pass

    def make_transition(self):
        if self.agent_.current_robot_state_ == robotModeMsg.MODE_TERMINATING:
            self.agent_.end_exploration()
            self.agent_.preempt_end_effector_planner()
            self.agent_.park_end_effector_planner()
            self.agent_.new_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.notify()
            self.agent_.current_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.release()
            self.agent_.current_robot_state_cond_.wait()
            self.agent_.current_robot_state_cond_.release()
            exit(0)
        elif self.agent_.current_robot_state_ == \
                robotModeMsg.MODE_TELEOPERATED_LOCOMOTION or \
            self.agent_.current_robot_state_ == \
                robotModeMsg.MODE_SEMI_AUTONOMOUS:
            self.agent_.preempt_move_base()
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
            self.agent_.preempt_move_base()
            self.agent_.preempt_end_effector_planner()
            self.agent_.park_end_effector_planner()
            self.agent_.new_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.notify()
            self.agent_.current_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.release()
            self.agent_.current_robot_state_cond_.wait()
            self.agent_.current_robot_state_cond_.release()
            return self.next_states_[1]

        if self.agent_.move_base_ac_.get_state() == GoalStatus.SUCCEEDED:
            self.agent_.preempt_end_effector_planner()
            self.agent_.park_end_effector_planner()
            self.agent_.new_robot_state_cond_.acquire()
            self.agent_.transition_to_state(robotModeMsg.
                                            MODE_TELEOPERATED_LOCOMOTION)
            self.agent_.new_robot_state_cond_.wait()
            self.agent_.new_robot_state_cond_.notify()
            self.agent_.current_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.release()
            self.agent_.current_robot_state_cond_.wait()
            self.agent_.current_robot_state_cond_.release()
            return self.next_states_[0]
        elif self.agent_.move_base_ac_.get_state() == GoalStatus.ABORTED:
            rospy.loginfo("move base sent aborted")
            rospy.loginfo(self.agent_.move_base_ac_.get_goal_status_text())
            return self.next_states_[3]
        return self.next_states_[2]
