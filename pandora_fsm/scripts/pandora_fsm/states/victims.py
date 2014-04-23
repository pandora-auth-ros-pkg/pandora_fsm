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
import smach
import smach_ros

import pandora_fsm

from actionlib import GoalStatus

from smach import State, StateMachine
from smach_ros import SimpleActionState

from pandora_fsm.states.my_monitor_state import MyMonitorState
from pandora_fsm.states.my_simple_action_state import MySimpleActionState
from data_fusion_communications.msg import *
from fsm_communications.msg import ValidateVictimAction, ValidateVictimGoal, ValidateVictimResult 
from std_msgs.msg import Empty

	
victim_found_topic = '/data_fusion/alert_handler/victim_found'
victim_update_topic = '/data_fusion/alert_handler/victim_update'
delete_victim_topic = '/data_fusion/alert_handler/delete_current_victim'
validate_hole_topic = '/data_fusion/alert_handler/validate_current_hole'
victim_verification_topic = '/data_fusion/alert_handler/victim_verified'
gui_validation_topic = '/gui/validate_victim'


class MonitorVictimState(MyMonitorState):
	
	def __init__(self, input_keys=[], output_keys=[]):
		MyMonitorState.__init__(self, victim_found_topic, VictimFoundMsg, 
		self.monitor_cb, extra_outcomes=['camera','thermal'], in_keys=input_keys, out_keys=output_keys)
		
	def monitor_cb(self, userdata, msg):
		if msg.victimNotificationType == msg.TYPE_THERMAL:
			return 'thermal'
		elif msg.victimNotificationType == msg.TYPE_CAMERA:
			return 'camera'
		else:
			return None

class MonitorVictimUpdateState(MyMonitorState):
	
	def __init__(self):
		MyMonitorState.__init__(self, victim_update_topic, Empty,
		self.monitor_cb, extra_outcomes=['update_victim'])
		
	def monitor_cb(self, userdata, msg):
		return 'update_victim'
		
class DeleteVictimState(MySimpleActionState):
	
	def __init__(self):
		MySimpleActionState.__init__(self, delete_victim_topic, 
									DeleteCurrentVictimAction,
									goal=DeleteCurrentVictimGoal(),
									outcomes=['succeeded','preempted'])

class ValidateHoleState(MySimpleActionState):
	
	def __init__(self, val):
		
		hole_goal = ValidateCurrentHoleGoal(valid=val)
		
		MySimpleActionState.__init__(self, validate_hole_topic,
									ValidateCurrentHoleAction,
									outcomes=['succeeded','preempted'],
									goal=hole_goal)

class VictimVerificationState(MyMonitorState):
	
	def __init__(self):
		MyMonitorState.__init__(self, victim_verification_topic, VictimToFsmMsg,
		self.monitor_cb, extra_outcomes=['got_verification'], in_keys=['victim_info'], out_keys=['victim_info'])
	
	def monitor_cb(self, userdata, msg):
		userdata[0].victim_info = msg
		return 'got_verification'
		
class DataFusionHold(State):
	
	def __init__(self):
		State.__init__(self, outcomes=['time_out','preempted'])
		
	def execute(self, userdata):
		counter = 0
		while counter < 10:
			rospy.sleep(1)
			
			counter = counter + 1
			
			# Check for preempt
			if self.preempt_requested():
				self.service_preempt()
				return 'preempted'
		
		return 'time_out'
		

class ValidateVictimState(MySimpleActionState):
	
	def __init__(self):
		
		MySimpleActionState.__init__(self, gui_validation_topic, 
									ValidateVictimAction,
									goal_cb=self.goal_callback,
									outcomes=['valid','not_valid','preempted'],
									result_cb=self.result_callback,
									input_keys=['victim_info']
									)

	def goal_callback(self, userdata, goal):
		
		goal = ValidateVictimGoal();
		goal.victimFoundx = userdata.victim_info.x
		goal.victimFoundy = userdata.victim_info.y
		goal.probability = userdata.victim_info.probability
		goal.sensorIDsFound = userdata.victim_info.sensors
		return goal
		

	def result_callback(self, userdata, status, result):
		if status == GoalStatus.SUCCEEDED:
			if result.victimValid:
				return 'valid'
			else:
				return 'not_valid'
			
		
