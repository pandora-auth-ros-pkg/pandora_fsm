#!/usr/bin/env python
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
			
		
