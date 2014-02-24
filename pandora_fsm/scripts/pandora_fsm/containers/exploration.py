#!/usr/bin/env python
import roslib; roslib.load_manifest('pandora_fsm')
import rospy
import smach
import smach_ros

import pandora_fsm

from smach import State, StateMachine, Concurrence

from pandora_fsm.states.navigation import *
from pandora_fsm.states.victims import *
	
def simpleExplorationContainer():
	
	sm_simple_exploration = StateMachine(outcomes=['preempted'])
	
	with sm_simple_exploration:
		
		StateMachine.add(
		    'SELECT_TARGET',
		    SelectTargetState('explore'), 
		    transitions={
			'succeeded':'MOVE_BASE',
			'aborted':'SELECT_TARGET',
			'preempted':'preempted'}
		)
		
		StateMachine.add(
		    'MOVE_BASE',
		    MoveBaseState(), 
		    transitions={
			'succeeded':'SELECT_TARGET',
			'aborted':'SELECT_TARGET',
			'preempted':'preempted'}
		)
		
		
	return sm_simple_exploration
	

def explorationWithVictims():
	
	def _termination_cb(outcome_map):
		return True
		
		
	cc = Concurrence(
		outcomes=[
			'thermal_alert',
			'camera_alert',
			'preempted' ], 
		default_outcome = 'preempted',
		outcome_map = {
			'thermal_alert':{'VICTIM_MONITOR':'thermal'}, 
			'camera_alert':{'VICTIM_MONITOR':'camera'},
			'preempted':{'EXPLORE':'preempted','VICTIM_MONITOR':'preempted'} },
		child_termination_cb=_termination_cb)
		
		
	with cc:
		
		Concurrence.add('VICTIM_MONITOR', MonitorVictimState())
		
		Concurrence.add('EXPLORE', simpleExplorationContainer())
		
		
		
		
	return cc
	
