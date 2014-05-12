#!/usr/bin/env python

import roslib; roslib.load_manifest('pandora_fsm')
import rospy

from actionlib import *
from actionlib.msg import *

from state_manager_communications.msg import RobotModeAction, RobotModeResult, \
                                              robotModeMsg
from data_fusion_communications.msg import QrNotificationMsg
from fsm_communications.msg import *
from std_msgs.msg import Int32, Empty

state_changer_action_topic = '/robot/state/change'
robocup_score_topic = '/data_fusion/alert_handler/robocup_score'
valid_victims_topic = '/data_fusion/alert_handler/valid_victims_counter'
qr_notification_topic = '/data_fusion/alert_handler/qr_notification'
teleoperation_topic = '/robot/state/clients'
teleoperation_ended_topic = '/teleoperation_ended'
robot_reset_topic = '/robot_reset'
robot_restart_topic = '/robot_restart'
robot_started_topic = 'robot_started'
exploration_ended_topic = 'exploration_ended'
monitor_victim_ended_topic = 'monitor_victim_ended'
validate_victim_ended_topic = 'validate_victim_ended'
exploration_restart_topic = 'exploration_restart'
robot_turn_back_topic = 'robot_turn_back'
return_to_orange_topic = 'return_to_orange'

robot_start_topic = '/fsm/robot_start'
exploration_start_topic = '/fsm/exploration_start'
monitor_victim_start_topic = '/fsm/monitor_victim_start'
validate_victim_start_topic = '/fsm/validate_victim_start'
abort_fsm_topic = '/fsm/abort_fsm'

class Communications():
  
  def __init__(self, unit):
    
    self.test_passed_ = False
    
    if unit:
      self.start_subs_acs()
    
    self.qr_notification_pub_ = rospy.Publisher(qr_notification_topic,
                                                QrNotificationMsg)
    self.teleoperation_pub_ = rospy.Publisher(teleoperation_topic, robotModeMsg)
    self.teleoperation_ended_pub_ = rospy.Publisher(teleoperation_ended_topic,
                                                    Empty)
    self.robot_reset_pub_ = rospy.Publisher(robot_reset_topic, Empty)
    self.robot_restart_pub_ = rospy.Publisher(robot_restart_topic, Empty)
    self.robocup_score_pub_ = rospy.Publisher(robocup_score_topic, Int32)
    self.valid_victims_pub_ = rospy.Publisher(valid_victims_topic, Int32)
    
    self.state_changer_as_ = SimpleActionServer(state_changer_action_topic,
                                                RobotModeAction,
                                                execute_cb = \
                                                  self.state_changer_cb,
                                                auto_start = False)
    self.state_changer_as_.start()
    self.robot_turn_back_as_ = SimpleActionServer(robot_turn_back_topic,
                                                  RobotTurnBackAction,
                                                  execute_cb = \
                                                    self.robot_turn_back_cb,
                                                  auto_start = False)
    self.robot_turn_back_as_.start()
    self.return_to_orange_as_ = SimpleActionServer(return_to_orange_topic,
                                                    ReturnToOrangeAction,
                                                    execute_cb = \
                                                      self.return_to_orange_cb,
                                                    auto_start = False)
    self.return_to_orange_as_.start()
  
  def start_subs_acs(self):
    self.robot_start_sub_ = rospy.Subscriber(robot_start_topic, Empty,
                                              self.robot_start_cb)
    
    self.exploration_start_sub_ = rospy.Subscriber(exploration_start_topic,
                                                    Empty,
                                                    self.exploration_start_cb)
    
    self.monitor_victim_start_sub_ = \
      rospy.Subscriber(monitor_victim_start_topic, Empty,
                        self.monitor_victim_start_cb)
    
    self.validate_victim_start_sub_ = \
      rospy.Subscriber(validate_victim_start_topic, Empty,
                        self.validate_victim_start_cb)
    
    self.abort_fsm_sub_ = rospy.Subscriber(abort_fsm_topic, Empty,
                                            self.abort_fsm_cb)
    
    self.robot_started_ac_ = SimpleActionClient(robot_started_topic,
                                                RobotStartedAction)
    self.robot_started_ac_.wait_for_server()
    
    self.exploration_ended_ac_ = SimpleActionClient(exploration_ended_topic,
                                                    ExplorationEndedAction)
    self.exploration_ended_ac_.wait_for_server()
    
    self.monitor_victim_ended_ac_ = \
      SimpleActionClient(monitor_victim_ended_topic, MonitorVictimEndedAction)
    self.monitor_victim_ended_ac_.wait_for_server()
    
    self.validate_victim_ended_ac_ = \
      SimpleActionClient(validate_victim_ended_topic, ValidateVictimEndedAction)
    self.validate_victim_ended_ac_.wait_for_server
    
    self.exploration_restart_ac_ = SimpleActionClient(exploration_restart_topic,
                                                      ExplorationRestartAction)
    self.exploration_restart_ac_.wait_for_server()
  
  def robot_start_cb(self, msg):
    rospy.loginfo('robot_start_cb')
    self.test_passed_ = True
  
  def exploration_start_cb(self, msg):
    rospy.loginfo('exploration_start_cb')
    self.test_passed_ = True
    
  def monitor_victim_start_cb(self, msg):
    rospy.loginfo('monitor_victim_start_cb')
    self.test_passed_ = True
    
  def validate_victim_start_cb(self, msg):
    rospy.loginfo('validate_victim_start_cb')
    self.test_passed_ = True
    
  def abort_fsm_cb(self, msg):
    rospy.loginfo('abort_fsm_cb')
    self.test_passed_ = True
  
  def state_changer_cb(self, goal):
    rospy.loginfo('state_changer_cb')
    self.test_passed_ = True
    result = RobotModeResult()
    self.state_changer_as_.set_succeeded(result)
  
  def robot_turn_back_cb(self, goal):
    rospy.loginfo('robot_turn_back_cb')
    self.test_passed_ = True
    result = RobotTurnBackResult()
    self.robot_turn_back_as_.set_succeeded(result)
  
  def return_to_orange_cb(self, goal):
    rospy.loginfo('return_to_orange_cb')
    self.test_passed_ = True
    result = ReturnToOrangeResult()
    self.return_to_orange_as_.set_succeeded(result)
