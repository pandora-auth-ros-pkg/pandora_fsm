#!/usr/bin/env python
import roslib; roslib.load_manifest('pandora_fsm')
import rospy
import actionlib

from state_manager_communications.msg import RobotModeAction

class ChangeRobotModeActionStub:
  
  def __init__(self):
    self.change_robot_mode_stub_as_ = \
      actionlib.SimpleActionServer('/robot/state/change', RobotModeAction,
                                    self.execute_cb, False)
    self.change_robot_mode_stub_as_.start()
  
  def execute_cb(self, goal):
    rospy.sleep(2)
    
    if self.change_robot_mode_stub_as_.is_preempt_requested():
      self.change_robot_mode_stub_as_.set_preempted()
    
    self.change_robot_mode_stub_as_.set_succeeded()

if __name__ == '__main__':
  rospy.init_node('change_robot_mode_stub')
  ChangeRobotModeActionStub()
  rospy.spin()
