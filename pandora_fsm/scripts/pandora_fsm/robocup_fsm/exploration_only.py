#!/usr/bin/env python
import roslib
roslib.load_manifest('pandora_fsm')  
roslib.load_manifest('smach')
roslib.load_manifest('smach_ros')
import rospy
import smach
import smach_ros

import pandora_fsm

import threading

from smach import State, StateMachine

from pandora_fsm.states.navigation import *
from pandora_fsm.containers.exploration import *

def main():
	rospy.init_node('exploration_fsm')
	
	sm = StateMachine(outcomes=['succeeded','aborted','preempted'])
    
	with sm:
		StateMachine.add(
		    'INITIAL_TURN',
		    InitialTurnState(), 
		    transitions={
			'succeeded':'EXPLORATION',
			'aborted':'EXPLORATION',
			'preempted':'preempted'}
		)
			
		StateMachine.add(
		    'EXPLORATION',
		    simpleExplorationContainer(), 
		    transitions={
			'preempted':'preempted'}
		)


	sis = smach_ros.IntrospectionServer('fsm_introspection', sm, '/EXPLORATION_FSM')
	
	sis.start()
    
	smach_ros.set_preempt_handler(sm)

	# Execute SMACH tree in a separate thread so that we can ctrl-c the script
	smach_thread = threading.Thread(target = sm.execute)
	smach_thread.start()
		
	rospy.spin()
	sis.stop()


if __name__ == '__main__':
	main()
