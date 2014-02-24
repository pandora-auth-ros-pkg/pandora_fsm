#!/usr/bin/env python
import roslib; roslib.load_manifest('pandora_fsm')
import rospy
import smach
import smach_ros

from target_selector_communications.msg import SelectTargetGoal, SelectTargetAction
from pandora_navigation_communications.msg import MoveBaseAction


from smach import StateMachine, Iterator
from smach_ros import SimpleActionState


def targetSelectionGoal(target_type):
	target_selection_goal = SelectTargetGoal()
	if target_type == 'explore':
		target_selection_goal.targetType = SelectTargetGoal.TYPE_EXPLORATION
	elif target_type == 'victim':
		target_selection_goal.targetType = SelectTargetGoal.TYPE_VICTIM
	else:
		rospy.logerr('Wrong target type!')
	return target_selection_goal
	
	
def TargetSelectorContainer(target_type):
	
	sm_target_selector = StateMachine(['target_sent','aborted','preempted'])
	
	with sm_target_selector:
		
		target_selection_goal = targetSelectionGoal(target_type)
		
		StateMachine.add(
			'GET_TARGET',
			SimpleActionState('/select_target',
				SelectTargetAction, 
				goal=target_selection_goal,
				result_key='target_pose'  ), 
			transitions={
				'succeeded':'MOVE_BASE',
				'aborted':'aborted',
				'preempted':'preempted'  }, 
			remapping={'target_pose':'next_target'}
		)
		
		StateMachine.add(
			'MOVE_BASE',
			SimpleActionState('/move_base',
				MoveBaseAction,
				goal_key='next_target'), 
			transitions={
				'succeeded':'target_sent',
				'aborted':'GET_TARGET',
				'preempted':'preempted' }
		)
		
	return sm_target_selector
	
def make_iterator(container, max_iter=1):
    
	it = Iterator(outcomes = ['preempted','aborted','time_out'],
						   it = lambda: range(0, max_iter),
						   it_label = 'index',
						   input_keys=[],
						   output_keys=[],
						   exhausted_outcome = 'time_out')
	with it:
						
		Iterator.set_contained_state('TARGET_CONTROLLER', container,  loop_outcomes=['target_sent'])

	return it

