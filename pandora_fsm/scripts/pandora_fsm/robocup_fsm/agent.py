#!/usr/bin/env python

import roslib; roslib.load_manifest('pandora_fsm')
import rospy
import pandora_fsm

from pandora_fsm.agent.agent_communications import *

def main():
  
  rospy.init_node('agent')
  
  agent = AgentCommunications()
  
  while not rospy.is_shutdown():
    agent.main()
  
  rospy.loginfo('agent terminated')

if __name__ == '__main__':
  main()
