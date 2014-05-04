#!/usr/bin/env python

import roslib; roslib.load_manifest('pandora_fsm')
import rospy
import pandora_fsm

from state_manager_communications.msg import RobotModeAction, RobotModeGoal, \
                                              robotModeMsg
from std_msgs.msg import Int32, Empty
from fsm_communications.msg import *
from data_fusion_communications.msg import QrNotificationMsg

from actionlib import *
from actionlib.msg import *

state_changer_action_topic = '/robot/state/change'
arena_type_topic = '/fsm/arena_type'
robocup_score_topic = '/data_fusion/alert_handler/robocup_score'
valid_victims_topic = '/data_fusion/alert_handler/valid_victims_counter'
qr_notification_topic = '/data_fusion/alert_handler/qr_notification'
teleoperation_topic = '/robot/state/clients'
teleoperation_ended_topic = '/teleoperation_ended'
robot_reset_topic = '/robot_reset'
robot_restart_topic = '/robot_restart'

robot_start_topic = '/fsm/robot_start'
exploration_start_topic = '/fsm/exploration_start'
monitor_victim_start_topic = '/fsm/monitor_victim_start'
validate_victim_start_topic = '/fsm/validate_victim_start'
abort_fsm_topic = '/fsm/abort_fsm'

class AgentCommunications():
  
  def __init__(self):
    rospy.Subscriber(arena_type_topic, ArenaTypeMsg, self.arena_type_cb)
    rospy.Subscriber(robocup_score_topic, Int32, self.score_cb)
    rospy.Subscriber(valid_victims_topic, Int32, self.valid_victims_cb)
    rospy.Subscriber(qr_notification_topic, QrNotificationMsg,
                      self.qr_notification_cb)
    rospy.Subscriber(teleoperation_topic, robotModeMsg, self.teleoperation_cb)
    rospy.Subscriber(teleoperation_ended_topic, Empty,
                      self.teleoperation_ended_cb)
    rospy.Subscriber(robot_reset_topic, Empty, self.robot_reset_cb)
    rospy.Subscriber(robot_restart_topic, Empty, self.robot_restart_cb)
    
    self.robot_start_pub_ = rospy.Publisher(robot_start_topic, Empty)
    self.exploration_start_pub_ = rospy.Publisher(exploration_start_topic, Empty)
    self.monitor_victim_start_pub_ = rospy.Publisher(monitor_victim_start_topic,
                                                      Empty)
    self.validate_victim_start_pub_ = \
      rospy.Publisher(validate_victim_start_topic, Empty)
    self.abort_fsm_pub_ = rospy.Publisher(abort_fsm_topic, Empty)
    
    self.state_changer_ac_ = SimpleActionClient(state_changer_action_topic,
                                                RobotModeAction)
    
    self.robot_started_as_ = SimpleActionServer('robot_started',
                                                RobotStartedAction,
                                                execute_cb =
                                                  self.robot_started_cb)
    self.exploration_ended_as_ = SimpleActionServer('exploration_ended',
                                                    ExplorationEndedAction,
                                                    execute_cb =
                                                      self.exploration_ended_cb)
    self.monitor_victim_ended_as_ = \
      SimpleActionServer('monitor_victim_ended', MonitorVictimEndedAction,
                          execute_cb = self.monitor_victim_ended_cb)
    self.validate_victim_ended_as_ = \
      SimpleActionServer('validate_victim_ended', ValidateVictimEndedAction,
                          execute_cb = self.validate_victim_ended_cb)
    
    self.exploration_restart_as_ = \
      SimpleActionServer('exploration_restart', ExplorationRestartAction,
                          execute_cb = self.exploration_restart_cb)
    
    self.current_arena_ = 1
    self.current_score_ = 0
    self.valid_victims_ = 0
    self.exploration_ = False
    self.teleoperation_ = False
    self.current_exploration_mode_ = 0
    self.qrs_ = 0
    
    self.robot_resets_ = 0
    self.robot_restarts_ = 0
    
    self.initial_time_ = rospy.get_rostime().secs
  
  def main(self):
    self.reset_robot_ = False
    self.start_robot()
    
    while not self.reset_robot_ and not rospy.is_shutdown():
      rospy.Rate(2).sleep()
      if not self.teleoperation_:
        if self.exploration_:
          self.validate_current_situation(self.current_arena_)
          rospy.loginfo('agent loop')
  
  def arena_type_cb(self, msg):
    if self.current_arena_ == msg.TYPE_YELLOW and \
            msg.arenaType == msg.TYPE_ORANGE:
      self.validate_current_situation(self, msg.arenaType)
  
  def qr_notification_cb(self, msg):
    self.qrs_ += 1
  
  def teleoperation_cb(self, msg):
    if msg.type == msg.TYPE_TRANSITION and \
        msg.mode == robotModeMsg.MODE_TELEOPERATED_LOCOMOTION:
      self.teleoperation_ = True
      self.abort_fsm_pub_.publish()
  
  def teleoperation_ended_cb(self, msg):
    self.teleoperation_ = False
  
  def robot_reset_cb(self, msg):
    self.current_score_ = 0
    self.valid_victims_ = 0
    self.exploration_ = False
    self.teleoperation_ = False
    self.current_exploration_mode_ = 0
    self.qrs_ = 0
    
    self.robot_resets_ += 1
    self.reset_robot_ = True
  
  def robot_restart_cb(self, msg):
    self.robot_restarts_ += 1
  
  def score_cb(self, msg):
    self.current_score_ = msg.data
  
  def valid_victims_cb(self, msg):
    self.valid_victims_ = msg.data
  
  def change_robot_state(self, new_state):
    mode_msg = robotModeMsg(nodeName = 'agent', mode = new_state)
    mode_goal = RobotModeGoal(modeMsg = mode_msg)
    
    self.state_changer_ac_.send_goal(mode_goal)
    self.state_changer_ac_.wait_for_result()
  
  def start_robot(self):
    rospy.Rate(10).sleep()
    self.robot_start_pub_.publish()
    rospy.Rate(0.5).sleep()
  
  def robot_started_cb(self, goal):
    rospy.loginfo('robot_started_cb')
    self.robot_started_as_.set_succeeded()
    rospy.Rate(1).sleep()
    self.exploration_ = True
    self.start_exploration(robotModeMsg.MODE_EXPLORATION, False)
  
  def exploration_ended_cb(self, goal):
    rospy.loginfo('exploration_ended_cb')
    self.exploration_ = False
    
    self.change_robot_state(robotModeMsg.MODE_IDENTIFICATION)
    
    self.exploration_ended_as_.set_succeeded()
    rospy.Rate(1).sleep()
    self.monitor_victim_start_pub_.publish()
  
  def monitor_victim_ended_cb(self, goal):
    rospy.loginfo('monitor_victim_ended_cb')
    self.monitor_victim_ended_as_.set_succeeded()
    rospy.Rate(1).sleep()
    self.validate_victim_start_pub_.publish()
  
  def validate_victim_ended_cb(self, goal):
    rospy.loginfo('validate_victim_ended_cb')
    self.change_robot_state(robotModeMsg.MODE_IDENTIFICATION)
    
    self.validate_victim_ended_as_.set_succeeded()
    rospy.Rate(1).sleep()
    self.monitor_victim_start_pub_.publish()
  
  def exploration_restart_cb(self, goal):
    rospy.loginfo('exploration_restart_cb')
    
    self.exploration_ = True
  
  def start_exploration(self, exploration_mode, restart):
    rospy.loginfo('start_exploration = %i' % exploration_mode)
    
    if restart:
      self.abort_fsm_pub_.publish()
    
    self.current_exploration_mode_ = exploration_mode
    self.change_robot_state(exploration_mode)
    rospy.Rate(1).sleep()
    self.exploration_start_pub_.publish()
  
  def validate_current_situation(self, arena_type):
    if arena_type == ArenaTypeMsg.TYPE_YELLOW:
      if rospy.get_rostime().secs - self.initial_time_ <= 600:
        if self.valid_victims_ == 3:
          if self.current_exploration_mode_ != robotModeMsg.MODE_FAST_EXPLORATION:
            self.start_exploration(robotModeMsg.MODE_FAST_EXPLORATION, True)
      elif rospy.get_rostime().secs - self.initial_time_ > 600 and \
            rospy.get_rostime().secs - self.initial_time_ <= 900:
        if self.valid_victims_ == 0:
          if self.current_exploration_mode_ != robotModeMsg.MODE_DEEP_EXPLORATION:
            self.start_exploration(robotModeMsg.MODE_DEEP_EXPLORATION, True)
        elif self.current_score_ <= 30:
          if self.current_exploration_mode_ != robotModeMsg.MODE_EXPLORATION:
            self.start_exploration(robotModeMsg.MODE_EXPLORATION, True)
        elif self.current_score_ > 30:
          if self.current_exploration_mode_ != robotModeMsg.MODE_FAST_EXPLORATION:
            self.start_exploration(robotModeMsg.MODE_FAST_EXPLORATION, True)
      elif rospy.get_rostime().secs - self.initial_time_ > 900:
        if self.valid_victims_ <= 1 and self.current_score_ <= 25:
          if self.current_exploration_mode_ != robotModeMsg.MODE_DEEP_EXPLORATION:
            self.start_exploration(robotModeMsg.MODE_DEEP_EXPLORATION, True)
        elif self.valid_victims_ <= 1 and self.current_score_ > 25:
          if self.current_exploration_mode_ != robotModeMsg.MODE_EXPLORATION:
            self.start_exploration(robotModeMsg.MODE_EXPLORATION, True)
        elif self.current_score_ <= 40:
          if self.current_exploration_mode_ != robotModeMsg.MODE_EXPLORATION:
            self.start_exploration(robotModeMsg.MODE_EXPLORATION, True)
        elif self.current_score_ > 40:
          if self.current_exploration_mode_ != robotModeMsg.MODE_FAST_EXPLORATION:
            self.start_exploration(robotModeMsg.MODE_FAST_EXPLORATION, True)
      if self.valid_victims_ == 4:
        if self.current_exploration_mode_ != robotModeMsg.MODE_FAST_EXPLORATION:
          self.start_exploration(robotModeMsg.MODE_FAST_EXPLORATION, True)
    elif arena_type == ArenaTypeMsg.TYPE_ORANGE:
      if self.valid_victims_ == 0:
        rospy.Rate(1).sleep()
