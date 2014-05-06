#!/usr/bin/env python
import roslib; roslib.load_manifest('pandora_fsm')
import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction

class MoveBaseActionStub:
  
  def __init__(self):
    self.move_base_stub_as_ = \
      actionlib.SimpleActionServer('/move_base', MoveBaseAction,
                                    self.execute_cb, False)
    self.move_base_stub_as_.start()
  
  def execute_cb(self, goal):
    rospy.sleep(10)
    
    if self.move_base_stub_as_.is_preempt_requested():
      self.move_base_stub_as_.set_preempted()
    
    self.move_base_stub_as_.set_succeeded()

if __name__ == '__main__':
  rospy.init_node('move_base_stub')
  MoveBaseActionStub()
  rospy.spin()
