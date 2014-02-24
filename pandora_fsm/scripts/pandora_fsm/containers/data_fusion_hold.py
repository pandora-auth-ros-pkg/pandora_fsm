#!/usr/bin/env python
import roslib; roslib.load_manifest('pandora_fsm')
import rospy
import smach
import smach_ros

import pandora_fsm

from smach import State, StateMachine, Concurrence

from pandora_fsm.states.victims import *
	
def dataFusionHold():
	
	def _termination_cb(outcome_map):
		return True
	
	sm  = StateMachine(outcomes=['verified','not_verified','preempted'], input_keys=['victim_info'], output_keys=['victim_info'])
	
	with sm:
		
		#~ sm.userdata.victim_info = None
	
		cc = Concurrence(
			outcomes=[
				'verified',
				'time_out',
				'preempted'], 
			default_outcome = 'time_out',
			outcome_map = {
				'verified':{'MONITOR_VERIFICATION':'got_verification'},
				'time_out':{'MONITOR_VERIFICATION':'preempted','TIMER':'time_out'},
				'preempted':{'TIMER':'preempted', 'MONITOR_VERIFICATION':'preempted'}},
			child_termination_cb=_termination_cb,
			input_keys=['victim_info'],
			output_keys=['victim_info'])
			
		with cc:
			
			Concurrence.add('MONITOR_VERIFICATION', VictimVerificationState(), remapping={'victim_info':'victim_info'})
			
			Concurrence.add('TIMER', DataFusionHold())
		
		StateMachine.add('WAIT_FOR_DF',
			cc,
			transitions={
			'time_out':'DELETE_CURRENT_HOLE',
			'verified':'verified',
			'preempted':'preempted'},
			remapping={'victim_info':'victim_info'}
		)
		
		StateMachine.add('DELETE_CURRENT_HOLE',
			ValidateHoleState(False),
			transitions={
			'succeeded':'not_verified',
			'preempted':'preempted'
			}
		)
	
	
	return sm
	

