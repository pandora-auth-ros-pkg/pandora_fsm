#!/usr/bin/env python
import roslib; roslib.load_manifest('pandora_fsm')
import rospy
import smach
import smach_ros

import pandora_fsm

from smach import State, StateMachine

from pandora_fsm.states.victims import *
	
def validateVictim():
	
	sm = StateMachine(outcomes=['valid','not_valid','preempted'], input_keys=['victim_info'])
		
	with sm:
		
		StateMachine.add(
			'VALIDATION_FROM_GUI',
			ValidateVictimState(),
			transitions={
			'valid':'VICTIM_TRUE',
			'not_valid':'VICTIM_FALSE',
			'preempted':'preempted'
			#'aborted':'VALIDATION_FROM_GUI'
			#~ 'succeeded':'VALIDATION_FROM_GUI'
			}
		)
		
		StateMachine.add(
			'VICTIM_TRUE',
			ValidateHoleState(True),
			transitions={
			'succeeded':'valid',
			'preempted':'preempted'
			}
		)
		
		StateMachine.add(
			'VICTIM_FALSE',
			ValidateHoleState(False),
			transitions={
			'succeeded':'not_valid',
			'preempted':'preempted'
			}
		)
		
	return sm

