#!/usr/bin/env python
import roslib; roslib.load_manifest('pandora_fsm')
import rospy
import smach
import smach_ros

from smach import State, StateMachine

from pandora_fsm.states.my_monitor_state import MyMonitorState
from pandora_fsm.states.my_simple_action_state import MySimpleActionState
from fsm_communications.msg import *
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
    self.current_arena = 1
    self.hazmats = 0
    self.eye_charts = 0
    self.motions = 0
    self.thermals = 0
    self.co = 0
    self.audio_vo = 0
    self.audio_ov = 0
    self.qrs = 0
  
  def arena_type_cb(self, msg):
    if current_arena == msg.arenaType:
      return None
    else:
      return 2
  
  def hazmat_cb(self, msg):
    self.hazmats += 1
    points = calculate_score()
  
  def eye_chart_cb(self, msg):
    self.eye_charts += 1
    points = calculate_score()
  
  def motion_cb(self, msg):
    self.motions += 1
    points = calculate_score()
  
  def thermal_cb(self, msg):
    self.thermals += 1
    points = calculate_score()
  
  def co_cb(self, msg):
    self.co += 1
    points = calculate_score()
  
  def audio_vo_cb(self, msg):
    self.audio_vo += 1
    points = calculate_score()
  
  def audio_ov_cb(self, msg):
    self.audio_ov += 1
    points = calculate_score()
  
  def qr_cb(self, msg):
    self.qrs += 1
    points = calculate_score()
  
  def calculate_score(self):
    points = (self.mazmats + self.eye_charts + self.motions + self.thermals +
              self.co + self.audio_vo + self.audio_ov) * 5 + self.qrs
    return points
