#!/usr/bin/env python

import roslib; roslib.load_manifest('pandora_fsm')
import rospy
import pandora_fsm

from pandora_fsm.agent.agent_communications import *

def main():
  
  rospy.init_node('agent')
  
  agent = AgentCommunications()
  agent.execute()
  #~ sis = smach_ros.IntrospectionServer('fsm_agent', sm_arena, '/PANDORA_FSM')
  #~ 
  #~ sis.start()
  #~ 
  #~ smach_thread = threading.Thread(target = sm_arena.execute)
  #~ smach_thread.start()
  #~ rospy.sleep(10)
  rospy.spin()
  #~ sis.stop()

if __name__ == '__main__':
	main()
