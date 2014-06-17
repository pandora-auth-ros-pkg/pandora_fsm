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
import cost_function

from math import exp, log, fabs


class ExplorationModeCostFunction(cost_function.CostFunction):

    def execute(self):
        time = float(rospy.get_rostime().secs - self.agent_.initial_time_)

        cost = self.agent_.valid_victims_ * \
            exp(4.2 - 0.3*self.agent_.max_victims_ +
                0.000111111*self.agent_.max_time_ -
                time/(2640 - 420*self.agent_.max_victims_ +
                      0.133333333*self.agent_.max_time_))

        cost += self.agent_.qrs_ * \
            exp(1.9 - 0.04*self.agent_.max_qrs_ +
                0.000444444*self.agent_.max_time_ -
                time/(300 - 24*self.agent_.max_qrs_ +
                      0.6*self.agent_.max_time_))

        cost += self.agent_.robot_resets_ * \
            exp(1.8 - 0.000444444*self.agent_.max_time_ +
                time/(600 + 0.6*self.agent_.max_time_))

        cost += self.agent_.robot_restarts_ * \
            (1 + exp(2.8 - 0.053333333*self.agent_.max_time_ -
                     time/(2 + 0.006666667*self.agent_.max_time_)))

        return cost


class ExplorationModeCostFunction2(cost_function.CostFunction):

    def __init__(self, agent):
        cost_function.CostFunction.__init__(self, agent)
        self.w1 = 1.8 - 0.3*self.agent_.max_victims_
        self.w2 = 0.24 - 0.008*self.agent_.max_qrs_
        self.w3 = 0.06 - 0.001333333*self.agent_.max_area_
        self.w4 = 0.15
        self.w5 = 0.05
        self.w6 = -(0.06 - 0.001333333*self.agent_.max_time_/60)
        self.sum_weights = self.w1 + self.w2 + self.w3 + \
            self.w4 + self.w5 + self.w6

    def execute(self):
        cost = self.agent_.valid_victims_ * self.w1
        cost += self.agent_.qrs_ * self.w2
        cost += self.agent_.qrs_ * self.w3
        cost += self.agent_.robot_resets_ * self.w4
        cost += self.agent_.robot_restarts_ * self.w5
        cost += (rospy.get_rostime().secs - self.agent_.initial_time_) * \
            self.w6/60
        cost /= self.sum_weights

        # < 1.2 DEEP
        # < 2.4 NORMAL
        return cost


class ExplorationModeCostFunction3(cost_function.CostFunction):

    def __init__(self, agent):
        cost_function.CostFunction.__init__(self, agent)
        self.w1 = 1.8 - 0.3*self.agent_.max_victims_
        self.w2 = 0.24 - 0.008*self.agent_.max_qrs_
        self.w3 = 0.06 - 0.001333333*self.agent_.max_area_
        self.w4 = 0.15
        self.w5 = 0.05
        self.sum_weights = self.w1 + self.w2 + self.w3 + self.w4 + self.w5

    def execute(self):
        cost = self.agent_.valid_victims_ * self.w1
        cost += self.agent_.qrs_ * self.w2
        cost += self.agent_.qrs_ * self.w3
        cost += self.agent_.robot_resets_ * self.w4
        cost += self.agent_.robot_restarts_ * self.w5
        cost /= self.sum_weights

        # < 1.4 DEEP
        # < 2.8 NORMAL
        return cost


class FindNewVictimToGoCostFunction(cost_function.CostFunction):

    def execute(self):
        cost = []
        for i in range(0, len(self.agent_.new_victims_)):
            cost.append(0)
            for aborted_victim in self.agent_.aborted_victims_:
                if self.agent_.\
                    calculate_distance_3d(self.agent_.new_victims_[i].
                                          victimPose.pose.position,
                                          aborted_victim[0].victimPose.
                                          pose.position) < \
                        self.agent_.aborted_victims_distance_:
                    cost[i] -= 2*aborted_victim[1]
            dist = \
                self.agent_.calculate_distance_2d(self.agent_.new_victims_[i].
                                                  victimPose.pose.position,
                                                  self.agent_.
                                                  current_robot_pose_.pose.
                                                  position)
            cost[i] += 10 - 5*dist
        return cost


class UpdateVictimCostFunction(cost_function.CostFunction):

    def execute(self):
        for victim in self.agent_.new_victims_:
            if victim.id == self.agent_.target_victim_.id:
                if fabs(self.agent_.target_victim_.probability -
                        victim.probability) > 0.001 or \
                    self.agent_.target_victim_.victimPose.pose.position.x != \
                        victim.victimPose.pose.position.x or \
                    self.agent_.target_victim_.victimPose.pose.position.y != \
                        victim.victimPose.pose.position.y or \
                    self.agent_.target_victim_.victimPose.pose.position.z != \
                        victim.victimPose.pose.position.z:
                    self.agent_.target_victim_ = victim
                    return 1
                return 0
