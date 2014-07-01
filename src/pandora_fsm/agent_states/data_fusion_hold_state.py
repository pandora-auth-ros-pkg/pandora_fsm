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
from pandora_data_fusion_msgs.msg import ValidateVictimGoal
from pandora_end_effector_planner.msg import MoveEndEffectorGoal


class DataFusionHoldState(state.State):

    def __init__(self, agent, next_states, cost_functions=None):
        state.State.__init__(self, agent, next_states, cost_functions)
        self.name_ = "data_fusion_hold_state"
        self.counter_ = 0

    def execute(self):
        self.data_fusion_hold()

    def make_transition(self):
        if self.agent_.current_robot_state_ == \
                robotModeMsg.MODE_TELEOPERATED_LOCOMOTION:
            self.agent_.end_effector_planner_ac_.cancel_all_goals()
            self.agent_.end_effector_planner_ac_.wait_for_result()
            goal = MoveEndEffectorGoal(command=MoveEndEffectorGoal.PARK)
            self.agent_.end_effector_planner_ac_.send_goal(goal)
            self.agent_.end_effector_planner_ac_.wait_for_result()
            self.counter_ = 0
            self.agent_.new_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.notify()
            self.agent_.current_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.release()
            self.agent_.current_robot_state_cond_.wait()
            self.agent_.current_robot_state_cond_.release()
            return self.next_states_[0]
        elif self.agent_.current_robot_state_ == robotModeMsg.MODE_OFF:
            self.agent_.end_effector_planner_ac_.cancel_all_goals()
            self.agent_.end_effector_planner_ac_.wait_for_result()
            goal = MoveEndEffectorGoal(command=MoveEndEffectorGoal.PARK)
            self.agent_.end_effector_planner_ac_.send_goal(goal)
            self.agent_.end_effector_planner_ac_.wait_for_result()
            self.counter_ = 0
            self.agent_.new_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.notify()
            self.agent_.current_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.release()
            self.agent_.current_robot_state_cond_.wait()
            self.agent_.current_robot_state_cond_.release()
            return self.next_states_[1]

        if self.counter_ == 10:
            self.counter_ = 0
            for victim in self.agent_.new_victims_:
                if victim.id == self.agent_.target_victim_.id:
                    face = False
                    for sensor in victim.sensors:
                        if sensor == 'FACE':
                            face = True
                    if victim.probability > \
                            self.agent_.valid_victim_probability_ and \
                            (len(victim.sensors) >= 2 or face):
                        self.agent_.target_victim_ = victim
                        return self.next_states_[3]
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
                            self.agent_.new_robot_state_cond_.acquire()
                            self.agent_.transition_to_state(robotModeMsg.
                                                            MODE_IDENTIFICATION)
                            self.agent_.new_robot_state_cond_.wait()
                            self.agent_.new_robot_state_cond_.notify()
                            self.agent_.current_robot_state_cond_.acquire()
                            self.agent_.new_robot_state_cond_.release()
                            self.agent_.current_robot_state_cond_.wait()
                            self.agent_.current_robot_state_cond_.release()
                            return self.next_states_[4]
                        self.agent_.end_effector_planner_ac_.cancel_all_goals()
                        self.agent_.end_effector_planner_ac_.wait_for_result()
                        goal = MoveEndEffectorGoal(command=MoveEndEffectorGoal.PARK)
                        self.agent_.end_effector_planner_ac_.send_goal(goal)
                        self.agent_.end_effector_planner_ac_.wait_for_result()
                        self.agent_.new_robot_state_cond_.acquire()
                        self.agent_.transition_to_state(robotModeMsg.
                                                        MODE_EXPLORATION)
                        self.agent_.new_robot_state_cond_.wait()
                        self.agent_.new_robot_state_cond_.notify()
                        self.agent_.current_robot_state_cond_.acquire()
                        self.agent_.new_robot_state_cond_.release()
                        self.agent_.current_robot_state_cond_.wait()
                        self.agent_.current_robot_state_cond_.release()
                        return self.next_states_[5]
        return self.next_states_[2]

    def data_fusion_hold(self):
        self.counter_ += 1
        rospy.sleep(1.)
