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


