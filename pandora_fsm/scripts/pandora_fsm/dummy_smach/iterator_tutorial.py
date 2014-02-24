#!/usr/bin/env python

import roslib; roslib.load_manifest('smach')
roslib.load_manifest('smach_ros')
import rospy

import smach
from smach import Iterator, StateMachine
from smach_ros import IntrospectionServer

import utils

def construct_sm():
    sm = StateMachine(outcomes = ['succeeded','aborted','preempted'])
    
    with sm:
        tutorial_it = Iterator(outcomes = ['succeeded','preempted','aborted'],
                               it = lambda: range(0, 10),
                               it_label = 'index',
                               input_keys=[],
                               output_keys=[],
                               exhausted_outcome = 'succeeded')
        with tutorial_it:
			
			container_sm =  utils.TargetSelectorContainer('explore')			
				
			#close container_sm
			Iterator.set_contained_state('CONTAINER_STATE', container_sm,  loop_outcomes=['target_sent'])
        
        #close the tutorial_it
        StateMachine.add('TUTORIAL_IT',tutorial_it,
                     {'succeeded':'succeeded',
                      'aborted':'aborted'})
    return sm

def main():
    rospy.init_node("iterator_tutorial")
    sm_iterator = construct_sm()

    # Run state machine introspection server for smach viewer
    intro_server = IntrospectionServer('iterator_tutorial',sm_iterator,'/ITERATOR_TUTORIAL')
    intro_server.start()


    outcome = sm_iterator.execute()

    rospy.spin()
    intro_server.stop()

if __name__ == "__main__":
    main()
