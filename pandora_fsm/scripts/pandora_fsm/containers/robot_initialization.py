#!/usr/bin/env python

import roslib; roslib.load_manifest('pandora_fsm')
import rospy
import smach
import smach_ros
import pandora_fsm

from smach import StateMachine
from pandora_fsm.states.state_changer import MonitorModeState, Timer
from pandora_fsm.states.navigation import InitialTurnState
from pandora_fsm.agent.agent_servers import RobotStart

from state_manager_communications.msg import robotModeMsg

def robotStart():
  
  sm = StateMachine(outcomes=['succeeded', 'preempted'])
  
  with sm:
    
    StateMachine.add(
      'ROBOT_START',
      RobotStart(),
      transitions={
        'succeeded':'MONITOR_START',
        'invalid':'ROBOT_START',
        'preempted':'preempted'
      }
    )
    
    StateMachine.add(
      'MONITOR_START',
      MonitorModeState(robotModeMsg.MODE_START_AUTONOMOUS),
      transitions={
        'invalid':'MONITOR_START',
        'valid':'WAIT_FOR_SLAM',
        'preempted':'preempted'
      }
    )
    
    StateMachine.add(
      'WAIT_FOR_SLAM',
      Timer(10),
      transitions={
        'time_out':'INITIAL_TURN',
        'preempted':'preempted'
      }
    )
    
    StateMachine.add(
      'INITIAL_TURN',
      InitialTurnState(),
      transitions={
        'succeeded':'succeeded',
        'aborted':'succeeded',
        'preempted':'preempted'
      }
    )
  
  return sm
