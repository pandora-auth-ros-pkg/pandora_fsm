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
# Author: Chris Zalidis

import roslib; roslib.load_manifest('pandora_fsm')
import rospy

from smach import State, StateMachine
from pandora_fsm.states.my_simple_action_state import MySimpleActionState
from pandora_fsm.states.my_monitor_state import MyMonitorState

from state_manager_communications.msg import *

state_changer_action_topic = '/robot/state/change'
state_monitor_topic = '/robot/state/clients'

class ChangeRobotModeState(MySimpleActionState):
	
	def __init__(self, state):
		mode_msg = robotModeMsg(nodeName='fsm', mode=state)
		mode_goal = RobotModeGoal(modeMsg=mode_msg)
		MySimpleActionState.__init__(self, state_changer_action_topic, RobotModeAction, goal=mode_goal, outcomes=['succeeded','preempted'])
		

class MonitorModeState(MyMonitorState):
	
	def __init__(self, mode):
		MyMonitorState.__init__(self, state_monitor_topic, robotModeMsg,
		self.monitor_cb, extra_outcomes=['valid'])
		self.mode_ = mode
		
	def monitor_cb(self, userdata, msg):
		if msg.type == msg.TYPE_TRANSITION and msg.mode == self.mode_:
			return 'valid'
		else:
			return None
			
class Timer(State):
	
	def __init__(self, time):
		State.__init__(self, outcomes=['time_out','preempted'])
		self.time_ = time 
		
	def execute(self, userdata):
		counter = 0
		while counter < 10:
			rospy.sleep(self.time_/10)
			
			counter = counter + 1
			
			# Check for preempt
			if self.preempt_requested():
				self.service_preempt()
				return 'preempted'
		
		return 'time_out'
	
