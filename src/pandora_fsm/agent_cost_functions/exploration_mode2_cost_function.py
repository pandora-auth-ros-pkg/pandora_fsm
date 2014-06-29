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


class ExplorationMode2CostFunction(cost_function.CostFunction):

    def __init__(self, agent):
        cost_function.CostFunction.__init__(self, agent)
        self.w1_ = 1.8 - 0.3*self.agent_.max_victims_
        self.w2_ = 0.24 - 0.008*self.agent_.max_qrs_
        self.w3_ = 0.06 - 0.001333333*self.agent_.max_yellow_area_
        self.w4_ = 0.15
        self.w5_ = 0.05
        self.w6_ = -(0.06 - 0.001333333*self.agent_.max_time_/60)
        self.sum_weights_ = self.w1_ + self.w2_ + self.w3_ + \
            self.w4_ + self.w5_ + self.w6_

    def execute(self):
        cost = self.agent_.valid_victims_ * self.w1_
        cost += self.agent_.qrs_ * self.w2_
        cost += self.agent_.yellow_arena_area_explored_ * self.w3_
        cost += self.agent_.robot_resets_ * self.w4_
        cost += self.agent_.robot_restarts_ * self.w5_
        cost += (rospy.get_rostime().secs - self.agent_.initial_time_) * \
            self.w6_/60
        cost /= self.sum_weights_

        # < 1.2 DEEP
        # < 2.4 NORMAL
        return cost
