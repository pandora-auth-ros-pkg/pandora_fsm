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

from fsm_communications.msg import *

def main():
  
  def termination_cb(outcome_map):
    return True
  
  def out_cb(outcome_map):
    if outcome_map['VICTIM_MONITORING'] == 'aborted':
      return 'monitoring_aborted'
    
    if outcome_map['ABORT_FSM'] == 'aborted':
      return 'aborted'
    
    return 'preempted'
  
  rospy.init_node('fsm')
  
  sm_everything = Concurrence(
    outcomes=[
      'monitoring_aborted',
      'aborted',
      'preempted'
    ],
    default_outcome='preempted',
    outcome_cb=out_cb,
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
  
  sm = StateMachine(outcomes=['preempted'])
  
  with sm:
    StateMachine.add(
      'PANDORA_FSM',
      sm_everything,
      transitions={
        'monitoring_aborted':'RESTART_EXPLORATION',
        'aborted':'PANDORA_FSM',
        'preempted':'preempted'
      }
    )
    
    StateMachine.add(
      'RESTART_EXPLORATION',
      MySimpleActionState('exploration_restart', ExplorationRestartAction,
                          goal=ExplorationRestartGoal(),
                          outcomes=['succeeded','preempted']),
      transitions={
        'succeeded':'PANDORA_FSM',
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
