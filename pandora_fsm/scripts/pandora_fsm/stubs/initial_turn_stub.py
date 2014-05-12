#!/usr/bin/env python
import roslib; roslib.load_manifest('pandora_fsm')
import rospy
import actionlib

from pandora_navigation_communications.msg import InitialTurnAction

class InitialTurnActionStub:
  
  def __init__(self):
    self.initial_turn_stub_as_ = actionlib.SimpleActionServer('/initial_turn',
                                                              InitialTurnAction,
                                                              self.execute_cb,
                                                              False)
    self.initial_turn_stub_as_.start()
  
  def execute_cb(self, goal):
    
    for i in range(100):
      rospy.Rate(10).sleep()
      
      if self.initial_turn_stub_as_.is_preempt_requested():
        self.initial_turn_stub_as_.set_preempted()
    
    self.initial_turn_stub_as_.set_succeeded()

def main():
  InitialTurnActionStub()
  rospy.spin()

if __name__ == '__main__':
  rospy.init_node('initial_turn_stub')
  main()
