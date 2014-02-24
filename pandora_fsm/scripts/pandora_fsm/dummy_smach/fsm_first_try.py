#!/usr/bin/env python
import roslib
roslib.load_manifest('pandora_fsm')  
roslib.load_manifest('smach')
roslib.load_manifest('smach_ros')
import rospy
import smach
import smach_ros

import threading

from pandora_navigation_communications.msg import MoveBaseAction
from pandora_navigation_communications.msg import InitialTurnAction

from smach import State, StateMachine
from smach_ros import SimpleActionState

import exploration
import identification

def main():
    rospy.init_node("fsm_dummy")

    sm = StateMachine(outcomes=['succeeded','aborted','preempted'])
    
    with sm:
	StateMachine.add(
	    'INITIAL_TURN',
	    SimpleActionState(
		'/initial_turn',
		 InitialTurnAction), 
	    transitions={
		'succeeded':'EXPLORATION',
		'aborted':'EXPLORATION',
		'preempted':'preempted'}
	)
        
	StateMachine.add(
	    'EXPLORATION',
	    exploration.ExplorationContainer(), 
	    transitions={
		'victim_thermal':'succeeded',
		'victim_camera':'IDENTIFICATION_TRACKING',
		'aborted':'aborted','preempted':'preempted',
		'time_out':'aborted'
	    }
	)
        
        #~ StateMachine.add('IDENTIFICATION_SIMPLE', identification.IdentificationSimpleContainer(), 
			#~ transitions={'parked':'succeeded','aborted':'aborted','preempted':'preempted'})
        #~ 
        StateMachine.add('IDENTIFICATION_TRACKING', identification.IdentificationTrackingContainer(), 
			transitions={'identification_finished':'succeeded','aborted':'EXPLORATION','preempted':'preempted'})
        #~ 
        #~ StateMachine.add('AGGRESSIVE_EXPLORATION', SimpleActionState('/navigation/initial_turn',
			#~ InitialTurnAction),
			#~ transitions={'succeeded':'succeeded','aborted':'aborted','preempted':'preempted'})

    sis = smach_ros.IntrospectionServer('smach_server', sm, '/DUMMY_FSM')
    sis.start()
    
    smach_ros.set_preempt_handler(sm)

    # Execute SMACH tree in a separate thread so that we can ctrl-c the script
    smach_thread = threading.Thread(target = sm.execute)
    smach_thread.start()
        
    rospy.spin()
    sis.stop()

if __name__=="__main__":
    main()

