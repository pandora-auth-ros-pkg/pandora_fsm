#!/usr/bin/env python
import roslib; roslib.load_manifest('pandora_fsm')
import rospy

from data_fusion_communications.msg import VictimFoundMsg

victim_found_topic = '/data_fusion/alert_handler/victim_found'

class MonitorVictimStub:
  
  def __init__(self):
    self.monitor_victim_stub_pub_ = rospy.Publisher(victim_found_topic,
                                                    VictimFoundMsg)
  
  def execute(self):
    rospy.Rate(10).sleep()
    
    msg = VictimFoundMsg(victimNotificationType = VictimFoundMsg.TYPE_CAMERA)
    self.monitor_victim_stub_pub_.publish(msg)

if __name__ == '__main__':
  rospy.init_node('monitor_victim_stub')
  MonitorVictimStub()
