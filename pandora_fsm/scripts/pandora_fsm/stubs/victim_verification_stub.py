#!/usr/bin/env python
import roslib; roslib.load_manifest('pandora_fsm')
import rospy

from data_fusion_communications.msg import VictimToFsmMsg

victim_verification_topic = '/data_fusion/alert_handler/victim_verified'

class VictimVerificationStub:
  
  def __init__(self):
    self.victim_verification_stub_pub_ = \
      rospy.Publisher(victim_verification_topic, VictimToFsmMsg)
  
  def execute(self):
    rospy.Rate(2).sleep()
    
    msg = VictimToFsmMsg()
    msg.x = 4
    msg.y = 5
    msg.probability = 0.5
    msg.sensors = ['1', '5']
    
    self.victim_verification_stub_pub_.publish(msg)

def main():
  stub = VictimVerificationStub()
  stub.execute()

if __name__ == '__main__':
  rospy.init_node('victim_verification_stub')
  main()
