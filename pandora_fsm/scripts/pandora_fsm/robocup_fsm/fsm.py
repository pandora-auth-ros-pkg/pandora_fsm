#!/usr/bin/env python

import roslib; roslib.load_manifest('pandora_fsm')
import rospy
import smach
import smach_ros
import pandora_fsm
import threading

from smach import StateMachine, Concurrence
from pandora_fsm.agent.agent_servers import AbortFSM
from pandora_fsm.containers.robot_initialization import *
from pandora_fsm.containers.exploration import *
from pandora_fsm.containers.monitor_victim import *
from pandora_fsm.containers.victim_validation import *

def main():
  
  def termination_cb(outcome_map):
    return True
  
  rospy.init_node('fsm')
  
  sm = StateMachine(outcomes=['preempted'])
  
  with sm:
    
    sm_everything = Concurrence(
      outcomes=[
        'succeeded',
        'aborted',
        'preempted'
      ],
      default_outcome='preempted',
      outcome_map={
        'succeeded':{'ROBOT_START':'succeeded'},
        'succeeded':{'EXPLORATION':'succeeded'},
        'succeeded':{'VICTIM_MONITORING':'succeeded'},
        'succeeded':{'VICTIM_MONITORING':'aborted'},
        'succeeded':{'VICTIM_VALIDATION':'valid'},
        'succeeded':{'VICTIM_VALIDATION':'not_valid'},
        'aborted':{'ABORT_FSM':'aborted'},
        'preempted':{'ROBOT_START':'preempted',
                      'EXPLORATION':'preempted',
                      'VICTIM_MONITORING':'preempted',
                      'VICTIM_VALIDATION':'preempted',
                      'ABORT_FSM':'preempted'}
      },
      child_termination_cb=termination_cb
    )
    
    sm_everything.userdata.victim_info = None
    
    with sm_everything:
      Concurrence.add('ROBOT_START', robotStart())
      Concurrence.add('EXPLORATION', explorationWithVictims())
      Concurrence.add('VICTIM_MONITORING', monitorVictim(),
                      remapping={'victim_info':'victim_info'})
      Concurrence.add('VICTIM_VALIDATION', validateVictim(),
                      remapping={'victim_info':'victim_info'})
      Concurrence.add('ABORT_FSM', AbortFSM())
    
    StateMachine.add(
      'PANDORA_FSM',
      sm_everything,
      transitions={
        'succeeded':'PANDORA_FSM',
        'aborted':'PANDORA_FSM',
        'preempted':'preempted'
      }
    )
  
  sis = smach_ros.IntrospectionServer('fsm_introspection', sm, '/PANDORA_FSM')
  sis.start()
  
  smach_ros.set_preempt_handler(sm)
  
  smach_thread = threading.Thread(target = sm.execute)
  smach_thread.start()
  
  rospy.spin()
  sis.stop()

if __name__ == '__main__':
	main()
