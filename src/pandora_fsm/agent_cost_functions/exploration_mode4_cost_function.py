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

from math import exp, log


class ExplorationMode4CostFunction(cost_function.CostFunction):

    def __init__(self, agent):
        cost_function.CostFunction.__init__(self, agent)
        self.w1_ = 0.5
        self.w2_ = 0.2
        self.w3_ = 0.2
        self.w4_ = 0.01
        self.w5_ = 0.003
        self.w6_ = 0.9
        self.sum_weights_ = self.w1_ + self.w2_ + self.w3_ + \
            self.w4_ + self.w5_ + self.w6_

    def execute(self):
        cost = float(self.agent_.valid_victims_ -
                     self.agent_.strategy4_previous_victims_) / \
            self.agent_.max_victims_ * self.w1_

        cost += float(self.agent_.qrs_ - self.agent_.strategy4_previous_qrs_) / \
            self.agent_.max_qrs_ * self.w2_

        cost += float(self.agent_.yellow_arena_area_explored_ -
                      self.agent_.strategy4_previous_area_) / \
            self.agent_.max_yellow_area_ * self.w3_

        cost += float(self.agent_.robot_resets_ -
                      self.agent_.strategy4_previous_resets_) * self.w4_

        cost += float(self.agent_.robot_restarts_ -
                      self.agent_.strategy4_previous_restarts_) * self.w5_

        cost += float(self.agent_.time_passed_ - rospy.get_rostime().secs - \
            self.agent_.initial_time_) / self.agent_.max_time_ * self.w6_

        cost /= float(self.sum_weights_)

        cost += self.agent_.strategy4_current_cost_

        return cost
