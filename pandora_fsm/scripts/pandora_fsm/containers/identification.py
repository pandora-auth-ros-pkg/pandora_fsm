#!/usr/bin/env python
import roslib; roslib.load_manifest('pandora_fsm')
import rospy
import smach
import smach_ros

import pandora_fsm

from smach import State, StateMachine, Concurrence

from pandora_fsm.states.navigation import *
from pandora_fsm.states.victims import *
from pandora_fsm.states.state_changer import *

from state_manager_communications.msg import robotModeMsg
	
def cameraIdentificationSimple():
	
	def _termination_cb(outcome_map):
		return True
	
	sm = StateMachine(outcomes=['victim_approached','aborted','preempted'])
	
	with sm:
		
		StateMachine.add('GET_VICTIM',
			SelectTargetState('victim'),
			transitions={
			'succeeded':'GO_TO_VICTIM',
			'aborted':'aborted',
			'preempted':'preempted'}
		)
		
		StateMachine.add('GO_TO_VICTIM',
			MoveBaseState(),
			transitions={
			'succeeded':'victim_approached',
			'aborted':'GET_VICTIM',
			'preempted':'preempted'}
		)
		
		
	return sm	
	
