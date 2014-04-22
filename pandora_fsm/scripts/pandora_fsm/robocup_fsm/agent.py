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
from pandora_fsm.states.state_changer import *
from pandora_fsm.states.arena_identification import *
from pandora_fsm.containers.exploration import *
from pandora_fsm.containers.identification import *
from pandora_fsm.containers.data_fusion_hold import *
from pandora_fsm.containers.victim_validation import *
from pandora_fsm.agent.agent_communications import *

from state_manager_communications.msg import robotModeMsg

def main():
  
  rospy.init_node('agent')
  
  agent = AgentCommunications()
  #~ sis = smach_ros.IntrospectionServer('fsm_agent', sm_arena, '/PANDORA_FSM')
  #~ 
  #~ sis.start()
  #~ 
  #~ smach_thread = threading.Thread(target = sm_arena.execute)
  #~ smach_thread.start()
  rospy.sleep(10)
  rospy.spin()
  #~ sis.stop()

if __name__ == '__main__':
	main()
