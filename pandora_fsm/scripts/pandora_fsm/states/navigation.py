#!/usr/bin/env python
import roslib; roslib.load_manifest('pandora_fsm')
import rospy
import smach
import smach_ros

from smach import State, StateMachine
from smach_ros import SimpleActionState

from target_selector_communications.msg import SelectTargetGoal, SelectTargetAction
from pandora_navigation_communications.msg import InitialTurnAction
from move_base_msgs.msg import MoveBaseAction
	
move_base_topic = '/move_base'
select_target_topic = '/select_target'
initial_turn_topic = '/initial_turn'

class MoveBaseState(SimpleActionState):
	
	def __init__(self):
		SimpleActionState.__init__(self, move_base_topic, MoveBaseAction, goal_key='target_pose')
		
		
class SelectTargetState(SimpleActionState):
	
	def __init__(self, target_type):
		
		target_selection_goal = self.targetSelectionGoal(target_type)
		
		SimpleActionState.__init__(self, select_target_topic,
				SelectTargetAction, 
				goal=target_selection_goal,
				result_key='target_pose'  ) 
		
	
	def targetSelectionGoal(self, target_type):
		
		target_selection_goal = SelectTargetGoal()
		
		if target_type == 'explore':
			target_selection_goal.targetType = SelectTargetGoal.TYPE_EXPLORATION
		elif target_type == 'victim':
			target_selection_goal.targetType = SelectTargetGoal.TYPE_VICTIM
		else:
			rospy.logerr('Wrong target type!')
		return target_selection_goal


class InitialTurnState(SimpleActionState):
	
	def __init__(self):
		SimpleActionState.__init__(self, initial_turn_topic, InitialTurnAction)


