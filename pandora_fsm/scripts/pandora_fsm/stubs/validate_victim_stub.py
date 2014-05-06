#!/usr/bin/env python
import roslib; roslib.load_manifest('pandora_fsm')
import rospy
import actionlib

from fsm_communications.msg import ValidateVictimAction, ValidateVictimResult

class ValidateVictimActionStub:
  
  def __init__(self):
    self.validate_victim_stub_as_ = \
      actionlib.SimpleActionServer('/gui/validate_victim', ValidateVictimAction,
                                    self.execute_cb, False)
    self.validate_victim_stub_as_.start()
  
  def execute_cb(self, goal):
    rospy.sleep(5)
    
    if self.validate_victim_stub_as_.is_preempt_requested():
      self.validate_victim_stub_as_.set_preempted()
    
    result = ValidateVictimResult(victimValid = True)
    #~ result = ValidateVictimResult(victimValid = False)
    self.validate_victim_stub_as_.set_succeeded(result)

if __name__ == '__main__':
  rospy.init_node('validate_victim_stub')
  ValidateVictimActionStub()
  rospy.spin()
