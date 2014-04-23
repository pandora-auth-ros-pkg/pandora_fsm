#!/usr/bin/env python

import roslib; roslib.load_manifest('pandora_fsm')
import rospy
import smach
import smach_ros
import pandora_fsm

from smach import State
from pandora_fsm.states.state_changer import ChangeRobotModeState

from fsm_communications.msg import RobotStartAction, ExplorationStartAction
from fsm_communications.msg import MonitorVictimStartAction, ValidateVictimStartAction
from state_manager_communications.msg import robotModeMsg

from actionlib import *
from actionlib.msg import *

class RobotStart(State):
  
  def __init__(self):
    State.__init__(self, outcomes=['succeeded','preempted'])
    self.as_ = SimpleActionServer('robot_start', RobotStartAction,
                                  execute_cb=self.execute_cb)
    self.executed_ = False
  
  def execute(self, userdata):
    self.r = rospy.Rate(10)
    while not rospy.is_shutdown():
      if self.executed_ == True:
        return 'succeeded'
      self.r.sleep()
  
  def execute_cb(self, msg):
    self.executed_ = True
    self.as_.set_succeeded()
    subscriber.unregister()

class ExplorationStart(State):
  
  def __init__(self):
    State.__init__(self, outcomes=['succeeded','preempted'])
    self.as_ = SimpleActionServer('exploration_start', ExplorationStartAction,
                                  execute_cb=self.execute_cb)
    self.executed_ = False
  
  def execute(self, userdata):
    self.r = rospy.Rate(10)
    while not rospy.is_shutdown():
      if self.executed_ == True:
        return 'succeeded'
      self.r.sleep()
  
  def execute_cb(self, msg):
    self.executed_ = True
    ChangeRobotModeState(robotModeMsg.MODE_EXPLORATION)
    self.as_.set_succeeded()
    subscriber.unregister()

class MonitorVictimStart(State):
  
  def __init__(self):
    State.__init__(self, outcomes=['succeeded','preempted'])
    self.as_ = SimpleActionServer('monitor_victim_start',
                                  MonitorVictimStartAction,
                                  execute_cb=self.execute_cb)
    self.executed_ = False
  
  def execute(self, userdata):
    self.r = rospy.Rate(10)
    while not rospy.is_shutdown():
      if self.executed_ == True:
        return 'succeeded'
      self.r.sleep()
  
  def execute_cb(self, msg):
    self.executed_ = True
    self.as_.set_succeeded()
    subscriber.unregister()

class ValidateVictimStart(State):
  
  def __init__(self):
    State.__init__(self, outcomes=['succeeded','preempted'])
    self.as_ = SimpleActionServer('validate_victim_start',
                                  ValidateVictimStartAction,
                                  execute_cb=self.execute_cb)
    self.executed_ = False
  
  def execute(self, userdata):
    self.r = rospy.Rate(10)
    while not rospy.is_shutdown():
      if self.executed_ == True:
        return 'succeeded'
      self.r.sleep()
  
  def execute_cb(self, msg):
    self.executed_ = True
    self.as_.set_succeeded()
    subscriber.unregister()
