#!/usr/bin/env python
import roslib; roslib.load_manifest('pandora_fsm')
import rospy
import actionlib

from target_selector_communications.msg import SelectTargetAction, \
                                                SelectTargetResult

class SelectTargetActionStub:
  
  def __init__(self):
    self.select_target_stub_as_ = \
      actionlib.SimpleActionServer('/select_target', SelectTargetAction,
                                    self.execute_cb, False)
    self.select_target_stub_as_.start()
  
  def execute_cb(self, goal):
    
    for i in range(20):
      rospy.Rate(10).sleep()
    
      if self.select_target_stub_as_.is_preempt_requested():
        self.select_target_stub_as_.set_preempted()
    
    result = SelectTargetResult()
    self.select_target_stub_as_.set_succeeded(result)

def main():
  SelectTargetActionStub()
  rospy.spin()

if __name__ == '__main__':
  rospy.init_node('select_target_stub')
  main()
