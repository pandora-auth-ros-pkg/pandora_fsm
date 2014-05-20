#!/usr/bin/env python

import roslib; roslib.load_manifest('pandora_fsm')
import rospy
import pandora_fsm

from state_manager_communications.msg import RobotModeAction, RobotModeGoal, \
                                              robotModeMsg
from std_msgs.msg import Int32, Empty
from fsm_communications.msg import *
from data_fusion_communications.msg import QrNotificationMsg
from math import exp, log

from dynamic_reconfigure.server import Server
from pandora_fsm.cfg import FSMParamsConfig

from actionlib import *
from actionlib.msg import *

state_changer_action_topic = '/robot/state/change'
robot_turn_back_topic = 'robot_turn_back'
exploration_mode_topic = 'exploration_mode'
arena_type_topic = '/arena_type'
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
    
    self.robot_started_as_ = SimpleActionServer('robot_started',
                                                RobotStartedAction,
                                                execute_cb =
                                                  self.robot_started_cb,
                                                auto_start = False)
    self.robot_started_as_.start()
    self.exploration_ended_as_ = SimpleActionServer('exploration_ended',
                                                    ExplorationEndedAction,
                                                    execute_cb =
                                                      self.exploration_ended_cb,
                                                    auto_start = False)
    self.exploration_ended_as_.start()
    self.monitor_victim_ended_as_ = \
      SimpleActionServer('monitor_victim_ended', MonitorVictimEndedAction,
                          execute_cb = self.monitor_victim_ended_cb,
                          auto_start = False)
    self.monitor_victim_ended_as_.start()
    self.validate_victim_ended_as_ = \
      SimpleActionServer('validate_victim_ended', ValidateVictimEndedAction,
                          execute_cb = self.validate_victim_ended_cb,
                          auto_start = False)
    self.validate_victim_ended_as_.start()
    self.exploration_restart_as_ = \
      SimpleActionServer('exploration_restart', ExplorationRestartAction,
                          execute_cb = self.exploration_restart_cb,
                          auto_start = False)
    self.exploration_restart_as_.start()
    
    self.state_changer_ac_ = SimpleActionClient(state_changer_action_topic,
                                                RobotModeAction)
    #~ self.state_changer_ac_.wait_for_server()
    self.robot_turn_back_ac_ = SimpleActionClient(robot_turn_back_topic,
                                                  RobotTurnBackAction)
    #~ self.robot_turn_back_ac_.wait_for_server()
    self.exploration_mode_ac_ = SimpleActionClient(exploration_mode_topic,
                                                    ExplorationModeAction)
    #~ self.exploration_mode_ac_.wait_for_server()
    
    Server(FSMParamsConfig, self.reconfigure)
    
    self.current_arena_ = ArenaTypeMsg.TYPE_YELLOW
    self.current_score_ = 0
    self.valid_victims_ = 0
    self.qrs_ = 0
    self.current_exploration_mode_ = 0
    self.exploration_ = False
    self.teleoperation_ = False
    self.turn_back_ = False
    
    self.robot_resets_ = 0
    self.robot_restarts_ = 0
  
  def main(self):
    while not rospy.is_shutdown():
      self.reset_robot_ = False
      self.start_robot()
      
      while not self.reset_robot_ and not rospy.is_shutdown():
        rospy.Rate(2).sleep()
        if not self.teleoperation_:
          if self.exploration_:
            self.evaluate_current_situation(self.current_arena_)
            rospy.loginfo('agent loop')
  
  def arena_type_cb(self, msg):
    rospy.loginfo('arena_type_cb')
    if self.current_arena_ == msg.TYPE_YELLOW and \
            msg.arenaType == msg.TYPE_ORANGE:
      self.evaluate_current_situation(msg.arenaType)
    if self.current_arena_ == msg.TYPE_ORANGE and \
            msg.arenaType == msg.TYPE_YELLOW_BLACK:
      self.evaluate_current_situation(msg.arenaType)
  
  def qr_notification_cb(self, msg):
    rospy.loginfo('qr_notification_cb')
    self.qrs_ += 1
  
  def teleoperation_cb(self, msg):
    rospy.loginfo('teleoperation_cb')
    if msg.type == msg.TYPE_TRANSITION and \
        msg.mode == robotModeMsg.MODE_TELEOPERATED_LOCOMOTION:
      self.teleoperation_ = True
      self.abort_fsm_pub_.publish()
  
  def teleoperation_ended_cb(self, msg):
    rospy.loginfo('teleoperation_ended_cb')
    self.teleoperation_ = False
  
  def robot_reset_cb(self, msg):
    rospy.loginfo('robot_reset_cb')
    self.reset_robot_ = True
    
    self.abort_fsm_pub_.publish()
    rospy.sleep(1.)
    
    self.current_score_ = 0
    self.valid_victims_ = 0
    self.qrs_ = 0
    self.current_exploration_mode_ = 0
    self.exploration_ = False
    self.teleoperation_ = False
    self.turn_back_ = False
    
    self.robot_resets_ += 1
  
  def robot_restart_cb(self, msg):
    rospy.loginfo('robot_restart_cb')
    self.robot_restarts_ += 1
  
  def score_cb(self, msg):
    rospy.loginfo('score_cb')
    self.current_score_ = msg.data
  
  def valid_victims_cb(self, msg):
    rospy.loginfo('valid_victims_cb')
    self.valid_victims_ = msg.data
  
  def change_robot_state(self, new_state):
    rospy.loginfo('change_robot_state')
    mode_msg = robotModeMsg(nodeName = 'agent', mode = new_state)
    mode_goal = RobotModeGoal(modeMsg = mode_msg)
    
    self.state_changer_ac_.send_goal(mode_goal)
    self.state_changer_ac_.wait_for_result()
  
  def start_robot(self):
    rospy.loginfo('start_robot')
    rospy.Rate(5).sleep()
    self.robot_start_pub_.publish()
  
  def robot_started_cb(self, goal):
    rospy.loginfo('robot_started_cb')
    rospy.Rate(5).sleep()
    self.start_exploration(ExplorationModeGoal.MODE_DEEP)
    self.exploration_ = True
    rospy.Rate(5).sleep()
    self.robot_started_as_.set_succeeded()
  
  def exploration_ended_cb(self, goal):
    rospy.loginfo('exploration_ended_cb')
    self.exploration_ = False
    self.current_exploration_mode_ = 0
    
    self.change_robot_state(robotModeMsg.MODE_IDENTIFICATION)
    
    rospy.Rate(5).sleep()
    self.monitor_victim_start_pub_.publish()
    rospy.Rate(5).sleep()
    self.exploration_ended_as_.set_succeeded()
  
  def monitor_victim_ended_cb(self, goal):
    rospy.loginfo('monitor_victim_ended_cb')
    rospy.Rate(5).sleep()
    self.validate_victim_start_pub_.publish()
    rospy.Rate(5).sleep()
    self.monitor_victim_ended_as_.set_succeeded()
  
  def validate_victim_ended_cb(self, goal):
    rospy.loginfo('validate_victim_ended_cb')
    
    self.validate_victim_ended_as_.set_succeeded()
    rospy.Rate(5).sleep()
    
    if self.turn_back_:
      goal = RobotTurnBackGoal(frontierExploration = True)
      self.robot_turn_back_ac_.send_goal(goal)
      self.robot_turn_back_ac_.wait_for_result()
      self.turn_back_ = False
    
    self.change_robot_state(robotModeMsg.MODE_IDENTIFICATION)
    rospy.Rate(5).sleep()
    self.monitor_victim_start_pub_.publish()
  
  def exploration_restart_cb(self, goal):
    rospy.loginfo('exploration_restart_cb')
    self.exploration_ = True
    rospy.Rate(5).sleep()
    self.exploration_restart_as_.set_succeeded()
  
  def start_exploration(self, exploration_mode):
    rospy.loginfo('start_exploration = %i' % exploration_mode)
    
    if self.current_exploration_mode_ != 0:
      self.abort_fsm_pub_.publish()
    
    rospy.Rate(2).sleep()
    self.current_exploration_mode_ = exploration_mode
    self.change_robot_state(robotModeMsg.MODE_EXPLORATION)
    goal = ExplorationModeGoal(explorationMode = exploration_mode)
    self.exploration_mode_ac_.send_goal(goal)
    self.exploration_mode_ac_.wait_for_result()
    self.exploration_start_pub_.publish()
  
  def cost_function(self, time):
    
    cost = self.valid_victims_ * \
      exp(5 - 0.3*self.max_victims_ - 0.000333333*self.max_time_ - \
        time/(1320 - 300*self.max_victims_ + 0.86666664*self.max_time_))
    
    cost += self.qrs_ * \
      exp(3.2 - 0.03*self.max_qrs_ - 0.000444444*self.max_time_ - \
        time/(-60 - 6*self.max_qrs_ + 0.6*self.max_time_))
    
    cost += self.robot_resets_ * \
      exp(1.8 - 0.000444444*self.max_time_ + time/(600 + 0.6*self.max_time_))
    
    cost += self.robot_restarts_ * \
      (1 + exp(2.5 - 0.000555556*self.max_time_ - time/(0.6*self.max_time_)))
    
    return cost
  
  def cost_function2(self):
    
    cost = self.valid_victims_ * 0.7
    cost += self.qrs_ * 0.08
    cost += self.robot_resets_ * 0.12
    cost += self.robot_restarts_ * 0.1
    
    if cost < 1.6:
      return 20
    elif cost < 2.4:
      return 30
    else:
      return 40
  
  def evaluate_current_situation(self, arena_type):
    if arena_type == ArenaTypeMsg.TYPE_YELLOW:
      current_cost = \
        self.cost_function(float(rospy.get_rostime().secs - self.initial_time_))
      #~ current_cost = cost_function2()
      if current_cost < 25:
        if self.current_exploration_mode_ != ExplorationModeGoal.MODE_DEEP:
          self.start_exploration(ExplorationModeGoal.MODE_DEEP)
      elif current_cost < 35:
        if self.current_exploration_mode_ != ExplorationModeGoal.MODE_NORMAL:
          self.start_exploration(ExplorationModeGoal.MODE_NORMAL)
      else:
        if self.current_exploration_mode_ != ExplorationModeGoal.MODE_FAST:
          self.start_exploration(ExplorationModeGoal.MODE_FAST)
    elif arena_type == ArenaTypeMsg.TYPE_ORANGE:
      if self.valid_victims_ == 0:
        goal = RobotTurnBackGoal(frontierExploration = False)
        self.robot_turn_back_ac_.send_goal(goal)
        self.robot_turn_back_ac_.wait_for_result()
        self.turn_back_ = True
      else:
        self.current_arena_ = ArenaTypeMsg.TYPE_ORANGE
        self.teleoperation_ = True
        self.abort_fsm_pub_.publish()
    elif arena_type == ArenaTypeMsg.TYPE_YELLOW_BLACK:
      rospy.Rate(1).sleep()
  
  def reconfigure(self, config, level):
    self.max_time_ = config["maxTime"]
    self.max_victims_ = config["arenaVictims"]
    self.max_qrs_ = config["maxQRs"]
    self.initial_time_ = rospy.get_rostime().secs - config["timePassed"]
    return config
