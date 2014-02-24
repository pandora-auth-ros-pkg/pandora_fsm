#!/usr/bin/env python
import roslib; roslib.load_manifest('pandora_fsm')
import rospy
import smach
import smach_ros

from data_fusion_communications.msg import VictimFoundMsg

from smach import State, StateMachine, Concurrence
from smach_ros import SimpleActionState
from my_monitor_state import MyMonitorState
from robotic_arm_communications.msg import MoveArmAction, MoveArmGoal

import utils

class MonitorVictimState(MyMonitorState):
	def __init__(self, input_keys=[], output_keys=[]):
		MyMonitorState.__init__(self,'/data_fusion/victim_found', VictimFoundMsg, 
		self.monitor_cb, ik=input_keys, ok=output_keys)
		
	def monitor_cb(self, userdata, msg):
		if msg.victimNotificationType == msg.TYPE_THERMAL or msg.victimNotificationType == msg.TYPE_CAMERA:
			userdata[0].victim_type = msg.victimNotificationType
			return False
		else:
			return True

class DecideVictimState(State):
	def __init__(self):
		State.__init__(self,outcomes=['thermal','camera'], input_keys=['victim_type'])
	
	def execute(self, userdata):
		victim_msg = VictimFoundMsg()
		if userdata.victim_type == victim_msg.TYPE_THERMAL:
			return 'thermal'
		elif userdata.victim_type == victim_msg.TYPE_CAMERA:
			return 'camera'
		else:
			rospy.logerr('Wrong message type!'+' victim_type = '+str(userdata.victim_type))

def _termination_cb(outcome_map):
	#~ print outcome_map
	if outcome_map['ARM_SCAN'] == 'succeeded' and outcome_map['VICTIM_MONITOR'] == None:
		return False
	else:
		return True

def ExplorationContainer():
	
	cc = Concurrence(
			outcomes=[
				'victim_thermal',
				'victim_camera',
				'time_out',
				'aborted',
				'preempted' ], 
			default_outcome = 'aborted',
			outcome_map = {
				'victim_thermal':{'VICTIM_MONITOR':'victim_thermal'}, 
				'victim_camera':{'VICTIM_MONITOR':'victim_camera'},
				'preempted':{'EXPLORE':'preempted','VICTIM_MONITOR':'preempted','ARM_SCAN':'preempted'}, 
				'aborted':{'EXPLORE':'aborted'}, 'time_out':{'EXPLORE':'time_out'} },
			child_termination_cb=_termination_cb)
	
	with cc:
		#~ Concurrence.add('TARGET_CONTROLLER', utils.TargetSelectorContainer('explore'))
		Concurrence.add(
			'EXPLORE',
			 #~ utils.make_iterator(utils.TargetSelectorContainer('explore'), max_iter=3)
			 utils.make_iterator(utils.TargetSelectorContainer('explore'), max_iter=100)
		)
		
		sm_victim_monitor = StateMachine(outcomes=['victim_thermal','victim_camera','preempted'])
		
		sm_victim_monitor.userdata.victim_type = 0
		
		with sm_victim_monitor:
			
			StateMachine.add('VICTIM_MONITORING', MonitorVictimState(
			input_keys=['victim_type'], output_keys=['victim_type']), 
			transitions={'invalid':'VICTIM_DECIDE', 'valid':'VICTIM_MONITORING', 'preempted':'preempted'}, 
			remapping={'victim_type':'victim_type'})
			
			StateMachine.add('VICTIM_DECIDE', DecideVictimState(), 
			transitions={'thermal':'victim_thermal','camera':'victim_camera'})
			
		Concurrence.add('VICTIM_MONITOR', sm_victim_monitor)
		
		sm_arm_scan = StateMachine(outcomes=['succeeded','preempted'])
		
		with sm_arm_scan:
			
			arm_goal = MoveArmGoal()
			arm_goal.motionType = arm_goal.TYPE_SCAN
			
			StateMachine.add('SCAN', 
				SimpleActionState(
				'/arm/arm_control',
				MoveArmAction,
				goal=arm_goal),
				transitions={
				'succeeded':'succeeded',
				'aborted':'SCAN',
				'preempted':'preempted'  }
			)
		
		Concurrence.add('ARM_SCAN', sm_arm_scan)
		
	return cc
	
	
