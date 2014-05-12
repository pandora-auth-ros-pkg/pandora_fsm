#!/usr/bin/env python
import roslib; roslib.load_manifest('pandora_fsm')
import rospy
import actionlib

from data_fusion_communications.msg import ValidateCurrentHoleAction, \
                                            ValidateCurrentHoleResult

class ValidateCurrentHoleActionStub:
  
  def __init__(self):
    self.validate_current_hole_stub_as_ = actionlib.\
      SimpleActionServer('/data_fusion/alert_handler/validate_current_hole',
                          ValidateCurrentHoleAction,
                          self.execute_cb, False)
    self.validate_current_hole_stub_as_.start()
  
  def execute_cb(self, goal):
    
    for i in range(20):
      rospy.Rate(10).sleep()
      
      if self.validate_current_hole_stub_as_.is_preempt_requested():
        self.validate_current_hole_stub_as_.set_preempted()
    
    result = ValidateCurrentHoleResult()
    self.validate_current_hole_stub_as_.set_succeeded(result)

def main():
  ValidateCurrentHoleActionStub()
  rospy.spin()

if __name__ == '__main__':
  rospy.init_node('validate_current_hole_stub')
  main()
