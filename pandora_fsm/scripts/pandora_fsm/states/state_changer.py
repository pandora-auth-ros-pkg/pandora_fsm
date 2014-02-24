#!/usr/bin/env python
import roslib; roslib.load_manifest('pandora_fsm')
import rospy

from smach import State, StateMachine
from pandora_fsm.states.my_simple_action_state import MySimpleActionState
from pandora_fsm.states.my_monitor_state import MyMonitorState

from state_manager_communications.msg import *

state_changer_action_topic = '/robot/state/change'
state_monitor_topic = '/robot/state/clients'

class ChangeRobotModeState(MySimpleActionState):
	
	def __init__(self, state):
		mode_msg = robotModeMsg(nodeName='fsm', mode=state)
		mode_goal = RobotModeGoal(modeMsg=mode_msg)
		MySimpleActionState.__init__(self, state_changer_action_topic, RobotModeAction, goal=mode_goal, outcomes=['succeeded','preempted'])
		

class MonitorModeState(MyMonitorState):
	
	def __init__(self, mode):
		MyMonitorState.__init__(self, state_monitor_topic, robotModeMsg,
		self.monitor_cb, extra_outcomes=['valid'])
		self.mode_ = mode
		
	def monitor_cb(self, userdata, msg):
		if msg.type == msg.TYPE_TRANSITION and msg.mode == self.mode_:
			return 'valid'
		else:
			return None
			
class Timer(State):
	
	def __init__(self, time):
		State.__init__(self, outcomes=['time_out','preempted'])
		self.time_ = time 
		
	def execute(self, userdata):
		counter = 0
		while counter < 10:
			rospy.sleep(self.time_/10)
			
			counter = counter + 1
			
			# Check for preempt
			if self.preempt_requested():
				self.service_preempt()
				return 'preempted'
		
		return 'time_out'
	
