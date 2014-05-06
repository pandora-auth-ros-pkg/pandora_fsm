#!/usr/bin/env python
import roslib; roslib.load_manifest('pandora_fsm')
import rospy

from std_msgs.msg import Empty

victim_update_topic = '/data_fusion/alert_handler/victim_update'

class MonitorVictimUpdateStub:
  
  def __init__(self):
    self.monitor_victim_update_stub_pub_ = rospy.Publisher(victim_update_topic,
                                                            Empty)
  
  def execute(self):
    rospy.Rate(10).sleep()
    
    self.monitor_victim_update_stub_pub_.publish()

if __name__ == '__main__':
  rospy.init_node('monitor_victim_update_stub')
  MonitorVictimUpdateStub()
