#!/usr/bin/env python

import roslib; roslib.load_manifest('pandora_fsm')
import rospy
import pandora_fsm

from fsm_communications.msg import *
#~ from fsm_communications.msg import RobotStartGoal
#~ from fsm_communications.msg import ExplorationStartGoal
from std_msgs.msg import Empty

from actionlib import *
from actionlib.msg import *

arena_type_topic = '/arena_type'
hazmat_topic = '/hazmat'
eye_chart_topic = '/eye_chart'
motion_topic = '/motion'
thermal_topic = '/thermal'
co_topic = '/co'
audio_vo_topic = '/audio_vo'
audio_ov_topic = '/audio_ov'
qr_topic = '/qr'

class AgentCommunications():
  
  def __init__(self):
    rospy.Subscriber(arena_type_topic, ArenaTypeMsg, self.arena_type_cb)
    rospy.Subscriber(hazmat_topic, Empty, self.hazmat_cb)
    rospy.Subscriber(eye_chart_topic, Empty, self.eye_chart_cb)
    rospy.Subscriber(motion_topic, Empty, self.motion_cb)
    rospy.Subscriber(thermal_topic, Empty, self.thermal_cb)
    rospy.Subscriber(co_topic, Empty, self.co_cb)
    rospy.Subscriber(audio_vo_topic, Empty, self.audio_vo_cb)
    rospy.Subscriber(audio_ov_topic, Empty, self.audio_ov_cb)
    rospy.Subscriber(qr_topic, Empty, self.qr_cb)
    self.current_arena_ = 1
    self.hazmats_ = 0
    self.eye_charts_ = 0
    self.motions_ = 0
    self.thermals_ = 0
    self.co_ = 0
    self.audio_vo_ = 0
    self.audio_ov_ = 0
    self.qrs_ = 0
  
  #~ def execute(self):
    #~ ac = SimpleActionClient('robot_start', Empty)
    #~ goal = RobotStartGoal()
    #~ ac.send_goal(goal)
    #~ ac.wait_for_result()
    #~ ac.get_result()
  
  def arena_type_cb(self, msg):
    if self.current_arena_ == msg.arenaType:
      return None
    else:
      return 2
  
  def hazmat_cb(self, msg):
    self.hazmats_ += 1
    points = calculate_score()
  
  def eye_chart_cb(self, msg):
    self.eye_charts_ += 1
    points = calculate_score()
  
  def motion_cb(self, msg):
    self.motions_ += 1
    points = calculate_score()
  
  def thermal_cb(self, msg):
    self.thermals_ += 1
    points = calculate_score()
  
  def co_cb(self, msg):
    self.co_ += 1
    points = calculate_score()
  
  def audio_vo_cb(self, msg):
    self.audio_vo_ += 1
    points = calculate_score()
  
  def audio_ov_cb(self, msg):
    self.audio_ov_ += 1
    points = calculate_score()
  
  def qr_cb(self, msg):
    self.qrs_ += 1
    points = calculate_score()
  
  def calculate_score(self):
    points = (self.mazmats_ + self.eye_charts_ + self.motions_ + self.thermals_ +
              self.co_ + self.audio_vo_ + self.audio_ov_) * 5 + self.qrs_
    return points
