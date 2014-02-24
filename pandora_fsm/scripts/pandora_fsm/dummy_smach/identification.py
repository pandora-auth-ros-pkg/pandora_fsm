#!/usr/bin/env python
import roslib; roslib.load_manifest('pandora_fsm')
import rospy
import smach
import smach_ros


from smach import State, StateMachine, Concurrence
from smach_ros import SimpleActionState, MonitorState
from target_selector_communications.msg import SelectTargetGoal, SelectTargetAction

from pandora_navigation_communications.msg import InitialTurnAction, MoveBaseAction
from robotic_arm_communications.msg import MoveArmAction, MoveArmGoal

import utils

def IdentificationSimpleContainer():
	
	sm_identification_simple = StateMachine(['parked','aborted','preempted'])
	
	with sm_identification_simple:
		
		StateMachine.add('GET_VICTIMS', SimpleActionState('/navigation/initial_turn', InitialTurnAction), 
		transitions={'succeeded':'GO_TO_VICTIM'})
		
		StateMachine.add('GO_TO_VICTIM', utils.TargetSelectorContainer('victim'), 
		transitions={'target_sent':'PARK','aborted':'aborted','preempted':'preempted'})
		
		StateMachine.add('PARK', SimpleActionState('/navigation/initial_turn', InitialTurnAction), 
		transitions={'succeeded':'parked','aborted':'aborted','preempted':'preempted'})
	
	return sm_identification_simple

def _termination_cb(outcome_map):
	#~ print outcome_map
	if outcome_map['ARM_FOLLOW_POINT'] == 'aborted' or outcome_map['ARM_FOLLOW_POINT'] == 'succeeded':
		return False
	else:
		return True


def IdentificationTrackingContainer():
	
	sm_identification_tracking = StateMachine(['identification_finished','aborted','preempted'])
	
	with sm_identification_tracking:
		
		
		target_selection_goal = SelectTargetGoal()
		arm_goal = MoveArmGoal()
		target_selection_goal.targetType = SelectTargetGoal.TYPE_VICTIM
		
		StateMachine.add(
			'GET_TARGET',
			SimpleActionState('/select_target',
				SelectTargetAction, 
				goal=target_selection_goal,
				result_key='target_pose'  ), 
			transitions={
				'succeeded':'GO_TO_VICTIM',
				'aborted':'aborted',
				'preempted':'preempted'  }
		)
		
		cc = Concurrence(
			outcomes=[
				'parked',
				'aborted',
				'preempted' ], 
			default_outcome = 'aborted',
			input_keys=['target_pose'],
			outcome_map = {
				'parked':{'MOVE_BASE':'succeeded'},
				'preempted':{'MOVE_BASE':'preempted','ARM_FOLLOW_POINT':'preempted'}, 
				'aborted':{'MOVE_BASE':'aborted','ARM_FOLLOW_POINT':'aborted'}},
			child_termination_cb=_termination_cb)
			
		with cc:
			
			Concurrence.add('MOVE_BASE', 
				SimpleActionState('/move_base',
				MoveBaseAction,
				goal_key='target_pose')
			)
			
			arm_goal.motionType = arm_goal.TYPE_FOLLOW_POINT
		
			Concurrence.add('ARM_FOLLOW_POINT', 
				SimpleActionState(
				'/arm/arm_control',
				MoveArmAction,
				goal=arm_goal)
			)
		
		
		StateMachine.add(
			'GO_TO_VICTIM',
			cc, 
			transitions={
				'parked':'ARM_APPROACH_POINT',
				'aborted':'aborted',
				'preempted':'preempted' },
			remapping={'target_pose':'target_pose'}
		)
		
		
		arm_goal.motionType = arm_goal.TYPE_APPROACH_POINT
		
		StateMachine.add('ARM_APPROACH_POINT', 
			SimpleActionState(
			'/arm/arm_control',
			MoveArmAction,
			goal=arm_goal),
			transitions={
			'succeeded':'identification_finished',
			'aborted':'ARM_APPROACH_POINT',
			'preempted':'preempted'  }
		)
	
	return sm_identification_tracking
	
	
	
