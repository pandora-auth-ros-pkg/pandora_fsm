#!/usr/bin/env python
import roslib; roslib.load_manifest('pandora_fsm')
import rospy

from state_manager_communications.msg import robotModeMsg

state_monitor_topic = '/robot/state/clients'

class MonitorStartStub:
  
  def __init__(self):
    self.monitor_start_stub_pub_ = rospy.Publisher(state_monitor_topic, robotModeMsg)
  
  def execute(self):
    rospy.Rate(2).sleep()
    
    msg = robotModeMsg()
    msg.type = msg.TYPE_TRANSITION
    msg.mode = msg.MODE_START_AUTONOMOUS
    
    self.monitor_start_stub_pub_.publish(msg)

def main():
  stub = MonitorStartStub()
  stub.execute()

if __name__ == '__main__':
  rospy.init_node('monitor_start_stub')
  main()
