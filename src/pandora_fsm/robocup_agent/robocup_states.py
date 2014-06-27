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

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from state_manager_communications.msg import robotModeMsg
from pandora_end_effector_planner.msg import MoveEndEffectorGoal
from pandora_rqt_gui.msg import ValidateVictimGUIGoal
from pandora_data_fusion_msgs.msg import ValidateVictimGoal, DeleteVictimGoal
from move_base_msgs.msg import MoveBaseGoal
from pandora_navigation_msgs.msg import ArenaTypeMsg, DoExplorationGoal


class WaitingToStartState(state.State):

    def __init__(self, agent, next_states, cost_functions=None):
        state.State.__init__(self, agent, next_states, cost_functions)
        self.name_ = "waiting_to_start_state"

    def execute(self):
        pass

    def make_transition(self):
        if self.agent_.current_robot_state_ == \
                robotModeMsg.MODE_TELEOPERATED_LOCOMOTION:
            self.agent_.new_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.notify()
            self.agent_.current_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.release()
            self.agent_.current_robot_state_cond_.wait()
            self.agent_.current_robot_state_cond_.release()
            return self.next_states_[0]
        elif self.agent_.current_robot_state_ == \
                robotModeMsg.MODE_START_AUTONOMOUS:
            self.agent_.new_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.notify()
            self.agent_.current_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.release()
            self.agent_.current_robot_state_cond_.wait()
            self.agent_.current_robot_state_cond_.release()
            return self.next_states_[2]
        else:
            return self.next_states_[1]


class TestAndParkEndEffectorPlannerState(state.State):

    def __init__(self, agent, next_states, cost_functions=None):
        state.State.__init__(self, agent, next_states, cost_functions)
        self.name_ = "test_and_park_end_effector_planner_state"

    def execute(self):
        self.test_and_park_end_effector_planner()

    def make_transition(self):
        if self.agent_.current_robot_state_ == \
                robotModeMsg.MODE_TELEOPERATED_LOCOMOTION:
            self.agent_.new_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.notify()
            self.agent_.current_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.release()
            self.agent_.current_robot_state_cond_.wait()
            self.agent_.current_robot_state_cond_.release()
            return self.next_states_[0]
        if self.agent_.end_effector_planner_ac_.get_state() == \
                actionlib.GoalStatus.ABORTED:
            self.agent_.new_robot_state_cond_.acquire()
            self.agent_.transition_to_state(robotModeMsg.MODE_OFF)
            self.agent_.new_robot_state_cond_.wait()
            self.agent_.new_robot_state_cond_.notify()
            self.agent_.current_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.release()
            self.agent_.current_robot_state_cond_.wait()
            self.agent_.current_robot_state_cond_.release()
            return self.next_states_[2]
        return self.next_states_[1]

    def test_and_park_end_effector_planner(self):
        goal = MoveEndEffectorGoal(command=MoveEndEffectorGoal.TEST)
        self.agent_.end_effector_planner_ac_.send_goal(goal)
        self.agent_.end_effector_planner_ac_.wait_for_result()
        if self.agent_.end_effector_planner_ac_.get_state() == \
                actionlib.GoalStatus.ABORTED:
            return None
        goal = MoveEndEffectorGoal(command=MoveEndEffectorGoal.PARK)
        self.agent_.end_effector_planner_ac_.send_goal(goal)
        self.agent_.end_effector_planner_ac_.wait_for_result()


class RobotStartState(state.State):

    def __init__(self, agent, next_states, cost_functions=None):
        state.State.__init__(self, agent, next_states, cost_functions)
        self.name_ = "robot_start_state"
        self.counter_ = 0

    def execute(self):
        self.wait_for_slam()

    def make_transition(self):
        if self.agent_.current_robot_state_ == \
                robotModeMsg.MODE_TELEOPERATED_LOCOMOTION:
            self.counter_ = 0
            self.agent_.new_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.notify()
            self.agent_.current_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.release()
            self.agent_.current_robot_state_cond_.wait()
            self.agent_.current_robot_state_cond_.release()
            return self.next_states_[0]
        elif self.agent_.current_robot_state_ == robotModeMsg.MODE_OFF:
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
            self.agent_.new_robot_state_cond_.acquire()
            self.agent_.transition_to_state(robotModeMsg.MODE_EXPLORATION)
            self.agent_.new_robot_state_cond_.wait()
            self.agent_.new_robot_state_cond_.notify()
            self.agent_.current_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.release()
            self.agent_.current_robot_state_cond_.wait()
            self.agent_.current_robot_state_cond_.release()
            return self.next_states_[3]
        return self.next_states_[2]

    def wait_for_slam(self):
        self.counter_ += 1
        rospy.sleep(1.)


class ScanEndEffectorPlannerState(state.State):

    def __init__(self, agent, next_states, cost_functions=None):
        state.State.__init__(self, agent, next_states, cost_functions)
        self.name_ = "scan_end_effector_planner_state"

    def execute(self):
        self.scan_end_effector_planner()

    def make_transition(self):
        if self.agent_.current_robot_state_ == \
                robotModeMsg.MODE_TELEOPERATED_LOCOMOTION:
            self.agent_.new_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.notify()
            self.agent_.current_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.release()
            self.agent_.current_robot_state_cond_.wait()
            self.agent_.current_robot_state_cond_.release()
            return self.next_states_[0]
        return self.next_states_[1]

    def scan_end_effector_planner(self):
        self.agent_.end_effector_planner_ac_.cancel_all_goals()
        self.agent_.end_effector_planner_ac_.wait_for_result()
        goal = MoveEndEffectorGoal(command=MoveEndEffectorGoal.SCAN)
        self.agent_.end_effector_planner_ac_.send_goal(goal)


class ExplorationStrategy1State(state.State):

    def __init__(self, agent, next_states, cost_functions=None):
        state.State.__init__(self, agent, next_states, cost_functions)
        self.name_ = "exploration_strategy1_state"

    def execute(self):
        pass

    def make_transition(self):
        if self.agent_.current_robot_state_ == \
                robotModeMsg.MODE_TELEOPERATED_LOCOMOTION:
            self.agent_.end_effector_planner_ac_.cancel_all_goals()
            self.agent_.end_effector_planner_ac_.wait_for_result()
            self.end_exploration()
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
            self.end_exploration()
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
            self.end_exploration()
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
                self.agent_.end_effector_planner_ac_.cancel_all_goals()
                self.agent_.end_effector_planner_ac_.wait_for_result()
                self.end_exploration()
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
            self.end_exploration()

        rospy.Rate(2).sleep()
        self.agent_.current_exploration_mode_ = exploration_mode
        goal = DoExplorationGoal(exploration_type=exploration_mode)
        self.agent_.do_exploration_ac_.send_goal(goal,
                                                 feedback_cb=self.feedback_cb,
                                                 done_cb=self.done_cb)

    def end_exploration(self):
        self.agent_.do_exploration_ac_.cancel_all_goals()
        self.agent_.do_exploration_ac_.wait_for_result()

    def feedback_cb(self, feedback):
        self.agent_.current_robot_pose_ = feedback.base_position

    def done_cb(self, status, result):
        self.agent_.current_exploration_mode_ = -1


class ExplorationStrategy2State(state.State):

    def __init__(self, agent, next_states, cost_functions=None):
        state.State.__init__(self, agent, next_states, cost_functions)
        self.name_ = "exploration_strategy2_state"

    def execute(self):
        pass

    def make_transition(self):
        if self.agent_.current_robot_state_ == \
                robotModeMsg.MODE_TELEOPERATED_LOCOMOTION:
            self.agent_.end_effector_planner_ac_.cancel_all_goals()
            self.agent_.end_effector_planner_ac_.wait_for_result()
            self.end_exploration()
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
            self.end_exploration()
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
            self.end_exploration()
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
            if current_cost < 1.2:
                if self.agent_.current_exploration_mode_ != \
                        DoExplorationGoal.TYPE_DEEP:
                    self.start_exploration(DoExplorationGoal.TYPE_DEEP)
            elif current_cost < 2.4:
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
                self.agent_.end_effector_planner_ac_.cancel_all_goals()
                self.agent_.end_effector_planner_ac_.wait_for_result()
                self.end_exploration()
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
            self.end_exploration()

        rospy.Rate(2).sleep()
        self.agent_.current_exploration_mode_ = exploration_mode
        goal = DoExplorationGoal(exploration_type=exploration_mode)
        self.agent_.do_exploration_ac_.send_goal(goal,
                                                 feedback_cb=self.feedback_cb,
                                                 done_cb=self.done_cb)

    def end_exploration(self):
        self.agent_.do_exploration_ac_.cancel_all_goals()
        self.agent_.do_exploration_ac_.wait_for_result()

    def feedback_cb(self, feedback):
        self.agent_.current_robot_pose_ = feedback.base_position

    def done_cb(self, status, result):
        self.agent_.current_exploration_mode_ = -1


class ExplorationStrategy3State(state.State):

    def __init__(self, agent, next_states, cost_functions=None):
        state.State.__init__(self, agent, next_states, cost_functions)
        self.name_ = "exploration_strategy3_state"

    def execute(self):
        pass

    def make_transition(self):
        if self.agent_.current_robot_state_ == \
                robotModeMsg.MODE_TELEOPERATED_LOCOMOTION:
            self.agent_.end_effector_planner_ac_.cancel_all_goals()
            self.agent_.end_effector_planner_ac_.wait_for_result()
            self.end_exploration()
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
            self.end_exploration()
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
            self.end_exploration()
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

        if rospy.get_rostime().secs - self.agent_.initial_time_ - \
                self.agent_.minutes_passed_ >= 60:

            for i in range((rospy.get_rostime().secs -
                            self.agent_.initial_time_ -
                            self.agent_.minutes_passed_) / 60):
                self.agent_.minutes_passed_ += 60
                self.agent_.strategy3_deep_limit_ = \
                    (1 + 0.135 - 0.003*self.agent_.max_time_/60) * \
                    self.agent_.strategy3_deep_limit_

            self.agent_.strategy3_fast_limit_ = \
                self.agent_.strategy3_deep_limit_ * 1.4

        if self.agent_.current_arena_ == ArenaTypeMsg.TYPE_YELLOW:
            if current_cost < self.agent_.strategy3_deep_limit_:
                if self.agent_.current_exploration_mode_ != \
                        DoExplorationGoal.TYPE_DEEP:
                    self.start_exploration(DoExplorationGoal.TYPE_DEEP)
            elif current_cost < self.agent_.strategy3_fast_limit_:
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
                self.agent_.end_effector_planner_ac_.cancel_all_goals()
                self.agent_.end_effector_planner_ac_.wait_for_result()
                self.end_exploration()
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
            self.end_exploration()

        rospy.Rate(2).sleep()
        self.agent_.current_exploration_mode_ = exploration_mode
        goal = DoExplorationGoal(exploration_type=exploration_mode)
        self.agent_.do_exploration_ac_.send_goal(goal,
                                                 feedback_cb=self.feedback_cb,
                                                 done_cb=self.done_cb)

    def end_exploration(self):
        self.agent_.do_exploration_ac_.cancel_all_goals()
        self.agent_.do_exploration_ac_.wait_for_result()

    def feedback_cb(self, feedback):
        self.agent_.current_robot_pose_ = feedback.base_position

    def done_cb(self, status, result):
        self.agent_.current_exploration_mode_ = -1


class ExplorationStrategy4State(state.State):

    def __init__(self, agent, next_states, cost_functions=None):
        state.State.__init__(self, agent, next_states, cost_functions)
        self.name_ = "exploration_strategy4_state"

    def execute(self):
        pass

    def make_transition(self):
        if self.agent_.current_robot_state_ == \
                robotModeMsg.MODE_TELEOPERATED_LOCOMOTION:
            self.agent_.end_effector_planner_ac_.cancel_all_goals()
            self.agent_.end_effector_planner_ac_.wait_for_result()
            self.end_exploration()
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
            self.end_exploration()
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
            self.end_exploration()
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

        if rospy.get_rostime().secs - self.agent_.initial_time_ - \
                self.agent_.minutes_passed_ >= 60:
            current_cost = self.cost_functions_[1].execute()
            self.agent_.strategy4_previous_victims_ = \
                self.agent_.valid_victims_
            self.agent_.strategy4_previous_qrs_ = self.agent_.qrs_
            self.agent_.strategy4_previous_area_ = \
                self.agent_.yellow_arena_area_explored_
            self.agent_.strategy4_previous_resets_ = self.agent_.robot_resets_
            self.agent_.strategy4_previous_restarts_ = \
                self.agent_.robot_restarts_
            self.agent_.strategy4_current_cost_ = current_cost
            self.agent_.minutes_passed_ = \
                rospy.get_rostime().secs - self.agent_.initial_time_

        if self.agent_.current_arena_ == ArenaTypeMsg.TYPE_YELLOW:
            if self.agent_.strategy4_current_cost_ < \
                    self.agent_.strategy4_deep_limit_:
                if self.agent_.current_exploration_mode_ != \
                        DoExplorationGoal.TYPE_DEEP:
                    self.start_exploration(DoExplorationGoal.TYPE_DEEP)
            elif self.agent_.strategy4_current_cost_ < \
                    self.agent_.strategy4_fast_limit_:
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
                self.agent_.end_effector_planner_ac_.cancel_all_goals()
                self.agent_.end_effector_planner_ac_.wait_for_result()
                self.end_exploration()
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
            self.end_exploration()

        rospy.Rate(2).sleep()
        self.agent_.current_exploration_mode_ = exploration_mode
        goal = DoExplorationGoal(exploration_type=exploration_mode)
        self.agent_.do_exploration_ac_.send_goal(goal,
                                                 feedback_cb=self.feedback_cb,
                                                 done_cb=self.done_cb)

    def end_exploration(self):
        self.agent_.do_exploration_ac_.cancel_all_goals()
        self.agent_.do_exploration_ac_.wait_for_result()

    def feedback_cb(self, feedback):
        self.agent_.current_robot_pose_ = feedback.base_position

    def done_cb(self, status, result):
        self.agent_.current_exploration_mode_ = -1


class ExplorationStrategy5State(state.State):

    def __init__(self, agent, next_states, cost_functions=None):
        state.State.__init__(self, agent, next_states, cost_functions)
        self.name_ = "exploration_strategy5_state"

    def execute(self):
        pass

    def make_transition(self):
        if self.agent_.current_robot_state_ == \
                robotModeMsg.MODE_TELEOPERATED_LOCOMOTION:
            self.agent_.end_effector_planner_ac_.cancel_all_goals()
            self.agent_.end_effector_planner_ac_.wait_for_result()
            self.end_exploration()
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
            self.end_exploration()
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
            self.end_exploration()
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

        if rospy.get_rostime().secs - self.agent_.initial_time_ - \
                self.agent_.minutes_passed_ >= 60:

            if self.agent_.valid_victims_ == 0:
                for i in range((rospy.get_rostime().secs -
                                self.agent_.initial_time_ -
                                self.agent_.minutes_passed_) / 60):
                    self.agent_.minutes_passed_ += 60
                    self.agent_.strategy5_deep_limit_ = \
                        (1 + 0.135 - 0.003*self.agent_.max_time_/60) * \
                        self.agent_.strategy5_deep_limit_

                self.agent_.strategy5_fast_limit_ = \
                    self.agent_.strategy5_deep_limit_ * 1.4
            elif self.agent_.valid_victims_ == 1 and \
                    rospy.get_rostime().secs - self.agent_.initial_time_ >= \
                    self.agent_.max_time_/2:
                for i in range((rospy.get_rostime().secs -
                                self.agent_.initial_time_ -
                                self.agent_.minutes_passed_) / 60):
                    self.agent_.minutes_passed_ += 60
                    self.agent_.strategy5_deep_limit_ = \
                        (1 + 0.09 - 0.002*self.agent_.max_time_/60) * \
                        self.agent_.strategy5_deep_limit_

                self.agent_.strategy5_fast_limit_ = \
                    self.agent_.strategy5_deep_limit_ * 1.4
            elif self.agent_.valid_victims_ == 1 and \
                    rospy.get_rostime().secs - self.agent_.initial_time_ < \
                    self.agent_.max_time_/2:
                for i in range((rospy.get_rostime().secs -
                                self.agent_.initial_time_ -
                                self.agent_.minutes_passed_) / 60):
                    self.agent_.minutes_passed_ += 60
                    self.agent_.strategy5_deep_limit_ = \
                        (1 + 0.06 - 0.001333333*self.agent_.max_time_/60) * \
                        self.agent_.strategy5_deep_limit_

                self.agent_.strategy5_fast_limit_ = \
                    self.agent_.strategy5_deep_limit_ * 1.4
            else:
                for i in range((rospy.get_rostime().secs -
                                self.agent_.initial_time_ -
                                self.agent_.minutes_passed_) / 60):
                    self.agent_.minutes_passed_ += 60
                    self.agent_.strategy5_deep_limit_ = \
                        (1 + 0.06 - 0.001333333*self.agent_.max_time_/60) * \
                        self.agent_.strategy5_deep_limit_

                self.agent_.strategy5_fast_limit_ = \
                    self.agent_.strategy5_deep_limit_ * 1.4

        if self.agent_.current_arena_ == ArenaTypeMsg.TYPE_YELLOW:
            if current_cost < self.agent_.strategy5_deep_limit_:
                if self.agent_.current_exploration_mode_ != \
                        DoExplorationGoal.TYPE_DEEP:
                    self.start_exploration(DoExplorationGoal.TYPE_DEEP)
            elif current_cost < self.agent_.strategy5_fast_limit_:
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
                self.agent_.end_effector_planner_ac_.cancel_all_goals()
                self.agent_.end_effector_planner_ac_.wait_for_result()
                self.end_exploration()
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
            self.end_exploration()

        rospy.Rate(2).sleep()
        self.agent_.current_exploration_mode_ = exploration_mode
        goal = DoExplorationGoal(exploration_type=exploration_mode)
        self.agent_.do_exploration_ac_.send_goal(goal,
                                                 feedback_cb=self.feedback_cb,
                                                 done_cb=self.done_cb)

    def end_exploration(self):
        self.agent_.do_exploration_ac_.cancel_all_goals()
        self.agent_.do_exploration_ac_.wait_for_result()

    def feedback_cb(self, feedback):
        self.agent_.current_robot_pose_ = feedback.base_position

    def done_cb(self, status, result):
        self.agent_.current_exploration_mode_ = -1


class OldExplorationState(state.State):

    def __init__(self, agent, next_states, cost_functions=None):
        state.State.__init__(self, agent, next_states, cost_functions)
        self.name_ = "old_exploration_state"

    def execute(self):
        pass

    def make_transition(self):
        if self.agent_.current_robot_state_ == \
                robotModeMsg.MODE_TELEOPERATED_LOCOMOTION:
            self.agent_.end_effector_planner_ac_.cancel_all_goals()
            self.agent_.end_effector_planner_ac_.wait_for_result()
            self.end_exploration()
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
            self.end_exploration()
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
            self.end_exploration()
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

        if self.agent_.current_exploration_mode_ != DoExplorationGoal.TYPE_DEEP:
            self.start_exploration(DoExplorationGoal.TYPE_DEEP)

        return self.next_states_[2]

    def start_exploration(self, exploration_mode):
        if self.agent_.current_exploration_mode_ != 0:
            self.end_exploration()

        rospy.Rate(2).sleep()
        self.agent_.current_exploration_mode_ = exploration_mode
        goal = DoExplorationGoal(explorationMode=exploration_mode)
        self.agent_.do_exploration_ac_.send_goal(goal,
                                                 feedback_cb=self.feedback_cb,
                                                 done_cb=self.done_cb)

    def end_exploration(self):
        self.agent_.do_exploration_ac_.cancel_all_goals()
        self.agent_.do_exploration_ac_.wait_for_result()

    def feedback_cb(self, feedback):
        self.agent_.current_robot_pose_ = feedback.base_position

    def done_cb(self, status, result):
        self.agent_.current_exploration_mode_ = -1


class TrackEndEffectorPlannerState(state.State):

    def __init__(self, agent, next_states, cost_functions=None):
        state.State.__init__(self, agent, next_states, cost_functions)
        self.name_ = "track_end_effector_planner_state"

    def execute(self):
        self.track_end_effector_planner()

    def make_transition(self):
        if self.agent_.current_robot_state_ == \
                robotModeMsg.MODE_TELEOPERATED_LOCOMOTION:
            self.agent_.new_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.notify()
            self.agent_.current_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.release()
            self.agent_.current_robot_state_cond_.wait()
            self.agent_.current_robot_state_cond_.release()
            return self.next_states_[0]
        return self.next_states_[1]

    def track_end_effector_planner(self):
        self.agent_.end_effector_planner_ac_.cancel_all_goals()
        self.agent_.end_effector_planner_ac_.wait_for_result()
        goal = MoveEndEffectorGoal()
        goal.command = MoveEndEffectorGoal.TRACK
        goal.point_of_interest = \
            self.agent_.target_victim_.victimPose.header.frame_id
        goal.center_point = "kinect_frame"
        self.agent_.end_effector_planner_ac_.send_goal(goal)


class IdentificationMoveToVictimState(state.State):

    def __init__(self, agent, next_states, cost_functions=None):
        state.State.__init__(self, agent, next_states, cost_functions)
        self.name_ = "identification_move_to_victim_state"

    def execute(self):
        self.move_to_victim()

    def make_transition(self):
        if self.agent_.current_robot_state_ == \
                robotModeMsg.MODE_TELEOPERATED_LOCOMOTION:
            self.agent_.end_effector_planner_ac_.cancel_all_goals()
            self.agent_.end_effector_planner_ac_.wait_for_result()
            self.agent_.new_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.notify()
            self.agent_.current_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.release()
            self.agent_.current_robot_state_cond_.wait()
            self.agent_.current_robot_state_cond_.release()
            return self.next_states_[0]
        return self.next_states_[1]

    def move_to_victim(self):
        victim = self.agent_.target_victim_.victimPose
        orientation = [victim.pose.orientation.x, victim.pose.orientation.y,
                       victim.pose.orientation.z, victim.pose.orientation.w]
        euler = euler_from_quaternion(orientation)
        x = euler[0]
        y = euler[1]
        z = euler[2]
        z = z + 180

        transformed_orientation = quaternion_from_euler(x, y, z)
        victim.pose.orientation.x = transformed_orientation[0]
        victim.pose.orientation.y = transformed_orientation[1]
        victim.pose.orientation.z = transformed_orientation[2]
        victim.pose.orientation.w = transformed_orientation[3]

        victim.pose.position.z = 0

        goal = MoveBaseGoal(target_pose=victim)
        self.agent_.move_base_ac_.send_goal(goal, feedback_cb=self.feedback_cb)

    def feedback_cb(self, feedback):
        self.agent_.current_robot_pose_ = feedback.base_position


class IdentificationCheckForVictimsState(state.State):

    def __init__(self, agent, next_states, cost_functions=None):
        state.State.__init__(self, agent, next_states, cost_functions)
        self.name_ = "identification_check_for_victims_state"

    def execute(self):
        pass

    def make_transition(self):
        if self.agent_.current_robot_state_ == \
                robotModeMsg.MODE_TELEOPERATED_LOCOMOTION:
            self.agent_.end_effector_planner_ac_.cancel_all_goals()
            self.agent_.end_effector_planner_ac_.wait_for_result()
            self.agent_.move_base_ac_.cancel_all_goals()
            self.agent_.move_base_ac_.wait_for_result()
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
            self.agent_.move_base_ac_.cancel_all_goals()
            self.agent_.move_base_ac_.wait_for_result()
            self.agent_.new_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.notify()
            self.agent_.current_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.release()
            self.agent_.current_robot_state_cond_.wait()
            self.agent_.current_robot_state_cond_.release()
            return self.next_states_[1]

        if self.agent_.end_effector_planner_ac_.get_state() == \
                actionlib.GoalStatus.ABORTED:
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

        if self.agent_.move_base_ac_.get_state() == \
                actionlib.GoalStatus.SUCCEEDED:
            self.agent_.new_robot_state_cond_.acquire()
            self.agent_.transition_to_state(robotModeMsg.MODE_DF_HOLD)
            self.agent_.new_robot_state_cond_.wait()
            self.agent_.new_robot_state_cond_.notify()
            self.agent_.current_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.release()
            self.agent_.current_robot_state_cond_.wait()
            self.agent_.current_robot_state_cond_.release()
            return self.next_states_[4]
        elif self.agent_.move_base_ac_.get_state() == \
                actionlib.GoalStatus.ABORTED:
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


class ValidationState(state.State):

    def __init__(self, agent, next_states, cost_functions=None):
        state.State.__init__(self, agent, next_states, cost_functions)
        self.name_ = "validation_state"

    def execute(self):
        self.reset_arena_type_to_yellow()
        self.validate_current_victim()

    def make_transition(self):
        if self.agent_.current_robot_state_ == \
                robotModeMsg.MODE_TELEOPERATED_LOCOMOTION:
            self.agent_.end_effector_planner_ac_.cancel_all_goals()
            self.agent_.end_effector_planner_ac_.wait_for_result()
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
            self.agent_.new_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.notify()
            self.agent_.current_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.release()
            self.agent_.current_robot_state_cond_.wait()
            self.agent_.current_robot_state_cond_.release()
            return self.next_states_[1]

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
            self.agent_.transition_to_state(robotModeMsg.MODE_IDENTIFICATION)
            self.agent_.new_robot_state_cond_.wait()
            self.agent_.new_robot_state_cond_.notify()
            self.agent_.current_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.release()
            self.agent_.current_robot_state_cond_.wait()
            self.agent_.current_robot_state_cond_.release()
            return self.next_states_[2]
        self.agent_.new_robot_state_cond_.acquire()
        self.agent_.transition_to_state(robotModeMsg.MODE_EXPLORATION)
        self.agent_.new_robot_state_cond_.wait()
        self.agent_.new_robot_state_cond_.notify()
        self.agent_.current_robot_state_cond_.acquire()
        self.agent_.new_robot_state_cond_.release()
        self.agent_.current_robot_state_cond_.wait()
        self.agent_.current_robot_state_cond_.release()
        return self.next_states_[3]

    def reset_arena_type_to_yellow(self):
        self.agent_.current_arena_ = ArenaTypeMsg.TYPE_YELLOW

    def validate_current_victim(self):
        goal = ValidateVictimGUIGoal()
        goal.victimFoundx = \
            self.agent_.target_victim_.victimPose.pose.position.x
        goal.victimFoundy = \
            self.agent_.target_victim_.victimPose.pose.position.y
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


class YellowBlackArenaSaveRobotPoseState(state.State):

    def __init__(self, agent, next_states, cost_functions=None):
        state.State.__init__(self, agent, next_states, cost_functions)
        self.name_ = "yellow_black_arena_save_robot_pose_state"

    def execute(self):
        self.agent_.current_exploration_mode_ = DoExplorationGoal.TYPE_FAST
        goal = DoExplorationGoal(exploration_type=DoExplorationGoal.TYPE_FAST)
        self.agent_.do_exploration_ac_.send_goal(goal,
                                                 feedback_cb=self.feedback_cb,
                                                 done_cb=self.done_cb)
        rospy.sleep(2.)
        self.agent_.do_exploration_ac_.cancel_all_goals()
        self.agent_.do_exploration_ac_.wait_for_result()

    def make_transition(self):
        if self.agent_.current_robot_state_ == \
                robotModeMsg.MODE_TELEOPERATED_LOCOMOTION:
            self.agent_.end_effector_planner_ac_.cancel_all_goals()
            self.agent_.end_effector_planner_ac_.wait_for_result()
            self.end_exploration()
            self.agent_.new_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.notify()
            self.agent_.current_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.release()
            self.agent_.current_robot_state_cond_.wait()
            self.agent_.current_robot_state_cond_.release()
            return self.next_states_[0]
        return self.next_states_[1]

    def end_exploration(self):
        self.agent_.do_exploration_ac_.cancel_all_goals()
        self.agent_.do_exploration_ac_.wait_for_result()

    def feedback_cb(self, feedback):
        self.agent_.save_robot_pose_ = feedback.base_position

    def done_cb(self, status, result):
        self.agent_.current_exploration_mode_ = -1


class YellowBlackArenaExplorationStrategy1State(state.State):

    def __init__(self, agent, next_states, cost_functions=None):
        state.State.__init__(self, agent, next_states, cost_functions)
        self.name_ = "yellow_black_arena_exploration_strategy1_state"

    def execute(self):
        pass

    def make_transition(self):
        if self.agent_.current_robot_state_ == \
                robotModeMsg.MODE_TELEOPERATED_LOCOMOTION:
            self.agent_.end_effector_planner_ac_.cancel_all_goals()
            self.agent_.end_effector_planner_ac_.wait_for_result()
            self.end_exploration()
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
            self.end_exploration()
            self.agent_.new_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.notify()
            self.agent_.current_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.release()
            self.agent_.current_robot_state_cond_.wait()
            self.agent_.current_robot_state_cond_.release()
            return self.next_states_[1]

        if self.agent_.yellow_black_arena_area_explored_ < \
                self.agent_.max_yellow_black_area_*0.75:
            if self.agent_.current_exploration_mode_ != \
                    DoExplorationGoal.TYPE_FAST:
                self.start_exploration(DoExplorationGoal.TYPE_FAST)
        else:
            if self.agent_.current_exploration_mode_ != \
                    DoExplorationGoal.TYPE_DEEP:
                self.start_exploration(DoExplorationGoal.TYPE_DEEP)
        return self.next_states_[2]

    def start_exploration(self, exploration_mode):
        if self.agent_.current_exploration_mode_ != -1:
            self.end_exploration()

        rospy.Rate(2).sleep()
        self.agent_.current_exploration_mode_ = exploration_mode
        goal = DoExplorationGoal(exploration_type=exploration_mode)
        self.agent_.do_exploration_ac_.send_goal(goal,
                                                 feedback_cb=self.feedback_cb,
                                                 done_cb=self.done_cb)

    def end_exploration(self):
        self.agent_.do_exploration_ac_.cancel_all_goals()
        self.agent_.do_exploration_ac_.wait_for_result()

    def feedback_cb(self, feedback):
        self.agent_.current_robot_pose_ = feedback.base_position

    def done_cb(self, status, result):
        self.agent_.current_exploration_mode_ = -1


class YellowBlackArenaTurnBackMoveBaseState(state.State):

    def __init__(self, agent, next_states, cost_functions=None):
        state.State.__init__(self, agent, next_states, cost_functions)
        self.name_ = "yellow_black_arena_turn_back_move_base_state"

    def execute(self):
        goal = MoveBaseGoal(target_pose=self.agent_.save_robot_pose_)
        self.agent_.move_base_ac_.send_goal(goal, feedback_cb=self.feedback_cb)

    def make_transition(self):
        if self.agent_.current_robot_state_ == \
                robotModeMsg.MODE_TELEOPERATED_LOCOMOTION:
            self.agent_.end_effector_planner_ac_.cancel_all_goals()
            self.agent_.end_effector_planner_ac_.wait_for_result()
            self.agent_.move_base_ac_.cancel_all_goals()
            self.agent_.move_base_ac_.wait_for_result()
            self.agent_.new_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.notify()
            self.agent_.current_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.release()
            self.agent_.current_robot_state_cond_.wait()
            self.agent_.current_robot_state_cond_.release()
            return self.next_states_[0]
        return self.next_states_[1]

    def feedback_cb(self, feedback):
        self.agent_.current_robot_pose_ = feedback.base_position


class YellowBlackArenaTurnBackCheckState(state.State):

    def __init__(self, agent, next_states, cost_functions=None):
        state.State.__init__(self, agent, next_states, cost_functions)
        self.name_ = "yellow_black_arena_turn_back_check_state"

    def execute(self):
        pass

    def make_transition(self):
        if self.agent_.current_robot_state_ == \
                robotModeMsg.MODE_TELEOPERATED_LOCOMOTION:
            self.agent_.end_effector_planner_ac_.cancel_all_goals()
            self.agent_.end_effector_planner_ac_.wait_for_result()
            self.agent_.move_base_ac_.cancel_all_goals()
            self.agent_.move_base_ac_.wait_for_result()
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
            self.agent_.move_base_ac_.cancel_all_goals()
            self.agent_.move_base_ac_.wait_for_result()
            self.agent_.new_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.notify()
            self.agent_.current_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.release()
            self.agent_.current_robot_state_cond_.wait()
            self.agent_.current_robot_state_cond_.release()
            return self.next_states_[1]

        if self.agent_.move_base_ac_.get_state() == \
                actionlib.GoalStatus.SUCCEEDED:
            self.agent_.end_effector_planner_ac_.cancel_all_goals()
            self.agent_.end_effector_planner_ac_.wait_for_result()
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
        elif self.agent_.move_base_ac_.get_state() == \
                actionlib.GoalStatus.ABORTED:
            return self.next_states_[3]
        return self.next_states_[2]


class TeleoperationState(state.State):

    def __init__(self, agent, next_states, cost_functions=None):
        state.State.__init__(self, agent, next_states, cost_functions)
        self.name_ = "teleoperation_state"

    def execute(self):
        pass

    def make_transition(self):
        if self.agent_.current_robot_state_ == robotModeMsg.MODE_OFF:
            self.agent_.new_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.notify()
            self.agent_.current_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.release()
            self.agent_.current_robot_state_cond_.wait()
            self.agent_.current_robot_state_cond_.release()
            return self.next_states_[1]
        elif self.agent_.current_robot_state_ == \
                robotModeMsg.MODE_START_AUTONOMOUS:
            self.agent_.new_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.notify()
            self.agent_.current_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.release()
            self.agent_.current_robot_state_cond_.wait()
            self.agent_.current_robot_state_cond_.release()

            for victim in self.agent_.new_victims_:
                goal = DeleteVictimGoal(victimId=victim_.id)
                self.agent_.delete_victim_ac_.send_goal(goal)
                self.agent_.delete_victim_ac_.wait_for_result()

            self.agent_.new_robot_state_cond_.acquire()
            self.agent_.transition_to_state(robotModeMsg.MODE_EXPLORATION)
            self.agent_.new_robot_state_cond_.wait()
            self.agent_.new_robot_state_cond_.notify()
            self.agent_.current_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.release()
            self.agent_.current_robot_state_cond_.wait()
            self.agent_.current_robot_state_cond_.release()
            return self.next_states_[2]
        return self.next_states_[0]


class StopButtonState(state.State):

    def __init__(self, agent, next_states, cost_functions=None):
        state.State.__init__(self, agent, next_states, cost_functions)
        self.name_ = "stop_button_state"

    def execute(self):
        pass

    def make_transition(self):
        if self.agent_.current_robot_state_ == \
                robotModeMsg.MODE_TELEOPERATED_LOCOMOTION:
            self.agent_.new_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.notify()
            self.agent_.current_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.release()
            self.agent_.current_robot_state_cond_.wait()
            self.agent_.current_robot_state_cond_.release()
            return self.next_states_[0]
        elif self.agent_.current_robot_state_ == \
                robotModeMsg.MODE_START_AUTONOMOUS:
            self.agent_.new_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.notify()
            self.agent_.current_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.release()
            self.agent_.current_robot_state_cond_.wait()
            self.agent_.current_robot_state_cond_.release()

            self.agent_.new_robot_state_cond_.acquire()
            self.agent_.transition_to_state(robotModeMsg.MODE_EXPLORATION)
            self.agent_.new_robot_state_cond_.wait()
            self.agent_.new_robot_state_cond_.notify()
            self.agent_.current_robot_state_cond_.acquire()
            self.agent_.new_robot_state_cond_.release()
            self.agent_.current_robot_state_cond_.wait()
            self.agent_.current_robot_state_cond_.release()
            return self.next_states_[2]
        return self.next_states_[1]
