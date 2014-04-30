#!/usr/bin/env python

import roslib; roslib.load_manifest('pandora_fsm')
import rospy
import smach
import smach_ros
import pandora_fsm

from smach import StateMachine, Concurrence
from pandora_fsm.agent.agent_servers import MonitorVictimStart
from pandora_fsm.states.state_changer import ChangeRobotModeState
from pandora_fsm.states.victims import MonitorVictimUpdateState
from pandora_fsm.containers.identification import *
from pandora_fsm.containers.data_fusion_hold import *

from state_manager_communications.msg import robotModeMsg
from fsm_communications.msg import MonitorVictimEndedAction, \
                                    MonitorVictimEndedGoal

def monitorVictim():
  
  def termination_cb(outcome_map):
    return True
  
  sm = StateMachine(outcomes=['succeeded', 'aborted', 'preempted'],
                              input_keys=['victim_info'],
                              output_keys=['victim_info'])
  
  with sm:
    
    StateMachine.add(
      'MONITOR_VICTIM_START',
      MonitorVictimStart(),
      transitions={
        'succeeded':'VICTIM_APPROACH',
        'invalid':'MONITOR_VICTIM_START',
        'preempted':'preempted'
      }
    )
    
    sm_victim = StateMachine(outcomes=['verified','not_verified',
                                        'preempted','aborted'],
                              input_keys=['victim_info'],
                              output_keys=['victim_info'])
    
    with sm_victim:
      
      StateMachine.add(
        'VICTIM_APPROACH',
        cameraIdentificationSimple(),
        transitions={
          'victim_approached':'ROBOT_MODE_DF_HOLD',
          'aborted':'aborted',
          'preempted':'preempted'
        }
      )
      
      StateMachine.add(
        'ROBOT_MODE_DF_HOLD',
        ChangeRobotModeState(robotModeMsg.MODE_DF_HOLD),
        transitions={
          'succeeded':'DATA_FUSION_HOLD',
          'preempted':'preempted'
        }
      )
      
      StateMachine.add(
        'DATA_FUSION_HOLD',
        dataFusionHold(),
        transitions={
          'verified':'verified',
          'not_verified':'not_verified',
          'preempted':'preempted'
        },
        remapping={'victim_info':'victim_info'}
      )
    
    cc = Concurrence(
      outcomes=[
        'verified',
        'not_verified',
        'update_victim',
        'preempted',
        'aborted'
      ],
      default_outcome='not_verified',
      input_keys=['victim_info'],
      output_keys=['victim_info'],
      outcome_map={
        'verified':{'VICTIM_IDENTIFICATION':'verified',
                    'VICTIM_UPDATE':'preempted'},
        'not_verified':{'VICTIM_IDENTIFICATION':'not_verified',
                        'VICTIM_UPDATE':'preempted'},
        'preempted':{'VICTIM_IDENTIFICATION':'preempted',
                      'VICTIM_UPDATE':'preempted'}, 
        'aborted':{'VICTIM_IDENTIFICATION':'aborted','VICTIM_UPDATE':'preempted'},
        'update_victim':{'VICTIM_UPDATE':'update_victim',
                          'VICTIM_IDENTIFICATION':'preempted'}
      },
      child_termination_cb=termination_cb
    )
    
    with cc:
      Concurrence.add('VICTIM_IDENTIFICATION', sm_victim, remapping={'victim_info':'victim_info'})
      Concurrence.add('VICTIM_UPDATE', MonitorVictimUpdateState())
    
    StateMachine.add(
      'MONITOR_VICTIM_AND_DO_WORK',
      cc,
      transitions={
        'verified':'MONITOR_VICTIM_ENDED',
        'not_verified':'MONITOR_VICTIM_AND_DO_WORK',
        'update_victim':'MONITOR_VICTIM_AND_DO_WORK',
        'preempted':'preempted',
        'aborted':'aborted'
      },
      remapping={'victim_info':'victim_info'}
    )
    
    StateMachine.add(
      'MONITOR_VICTIM_ENDED',
      MySimpleActionState('monitor_victim_ended', MonitorVictimEndedAction,
                          goal=MonitorVictimEndedGoal(),
                          outcomes=['succeeded','preempted']),
      transitions={
        'succeeded':'MONITOR_VICTIM_START',
        'preempted':'preempted'
      }
    )
  
  return sm
