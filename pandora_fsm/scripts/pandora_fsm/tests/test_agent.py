#!/usr/bin/env python

import roslib; roslib.load_manifest('pandora_fsm')
import sys
import rospy
import smach
import smach_ros
import unittest
import rostest

from actionlib import *
from actionlib.msg import *

from smach import StateMachine
from state_manager_communications.msg import RobotModeAction, RobotModeGoal, \
                                              robotModeMsg
from pandora_fsm.agent.agent_servers import *
from pandora_fsm.agent.agent_communications import *
from fsm_communications.msg import *
from data_fusion_communications.msg import QrNotificationMsg
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

class TestAgent(unittest.TestCase):
  
  def setUp(self):
    
    self.test_passed_ = False
    
    rospy.Subscriber(robot_start_topic, Empty, self.message_cb)
    rospy.Subscriber(exploration_start_topic, Empty, self.message_cb)
    rospy.Subscriber(monitor_victim_start_topic, Empty, self.message_cb)
    rospy.Subscriber(validate_victim_start_topic, Empty, self.message_cb)
    rospy.Subscriber(abort_fsm_topic, Empty, self.message_cb)
    
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
                                                execute_cb = self.action_cb)
    
    self.robot_turn_back_as_ = SimpleActionServer(robot_turn_back_topic,
                                                  RobotTurnBackAction,
                                                  execute_cb = self.action_cb)
    
    self.return_to_orange_as_ = SimpleActionServer(return_to_orange_topic,
                                                    RobotModeAction,
                                                    execute_cb = self.action_cb)
    
    self.test_agent_ = AgentCommunications()
    
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
    self.validate_victim_ended_ac_.wait_for_server()
    self.exploration_restart_ac_ = SimpleActionClient(exploration_restart_topic,
                                                      ExplorationRestartAction)
    self.exploration_restart_ac_.wait_for_server()
    rospy.sleep(2)
  
  def tearDown(self):
    self.test_agent_ = None
    self.test_passed_ = False
  
  def message_cb(self, msg):
    self.test_passed_ = True
  
  def action_cb(self, goal):
    self.test_passed_ = True
    self.state_changer_as_.set_succeeded()
  
  def test_cost_function(self):
    self.test_agent_.valid_victims_ = 2
    self.test_agent_.qrs_ = 10
    self.test_agent_.robot_resets_ = 1
    self.test_agent_.robot_restarts_ = 4
    cost = self.test_agent_.cost_function(600.0)
    self.assertEqual(cost, 64.6837664225596)
  
  def test_evaluate_current_situation_orange_turn_back(self):
    self.test_agent_.evaluate_current_situation(ArenaTypeMsg.TYPE_ORANGE)
    self.assertTrue(self.test_agent_.turn_back_)
  
  def test_evaluate_current_situation_orange_teleoperation(self):
    self.test_agent_.valid_victims_ = 1
    self.test_agent_.evaluate_current_situation(ArenaTypeMsg.TYPE_ORANGE)
    self.assertTrue(self.test_agent_.teleoperation_)
    self.assertEqual(self.test_agent_.current_arena_, ArenaTypeMsg.TYPE_ORANGE)
  
  def test_evaluate_current_situation_yellow_normal(self):
    self.test_agent_.initial_time_ = rospy.get_rostime().secs - 600
    self.test_agent_.valid_victims_ = 1
    self.test_agent_.qrs_ = 6
    self.test_agent_.robot_resets_ = 0
    self.test_agent_.robot_restarts_ = 1
    self.test_agent_.evaluate_current_situation(ArenaTypeMsg.TYPE_YELLOW)
    self.assertEqual(self.test_agent_.current_exploration_mode_,
                      robotModeMsg.MODE_EXPLORATION)
  
  def test_evaluate_current_situation_yellow_fast(self):
    self.test_agent_.initial_time_ = rospy.get_rostime().secs - 600
    self.test_agent_.valid_victims_ = 2
    self.test_agent_.qrs_ = 10
    self.test_agent_.robot_resets_ = 1
    self.test_agent_.robot_restarts_ = 4
    self.test_agent_.evaluate_current_situation(ArenaTypeMsg.TYPE_YELLOW)
    self.assertEqual(self.test_agent_.current_exploration_mode_,
                      robotModeMsg.MODE_FAST_EXPLORATION)
  
  def test_exploration_ended(self):
    self.test_agent_.exploration_ = True
    goal = ExplorationEndedGoal()
    self.exploration_ended_ac_.send_goal(goal)
    self.exploration_ended_ac_.wait_for_result()
    self.assertFalse(self.test_agent_.exploration_)
    self.assertTrue(self.test_passed_)
  
  def test_exploration_restart(self):
    goal = ExplorationRestartGoal()
    self.exploration_restart_ac_.send_goal(goal)
    self.exploration_restart_ac_.wait_for_result()
    self.assertTrue(self.test_agent_.exploration_)
  
  def test_monitor_victim_ended(self):
    goal = MonitorVictimEndedGoal()
    self.monitor_victim_ended_ac_.send_goal(goal)
    self.monitor_victim_ended_ac_.wait_for_result()
    self.assertTrue(self.test_passed_)
  
  def test_qr_notification(self):
    msg = QrNotificationMsg()
    self.qr_notification_pub_.publish(msg)
    rospy.sleep(1)
    self.assertEqual(self.test_agent_.qrs_, 1)
  
  def test_robocup_score(self):
    msg = Int32(data = 25)
    self.robocup_score_pub_.publish(msg)
    rospy.sleep(1)
    self.assertEqual(self.test_agent_.current_score_, 25)
  
  def test_robot_reset(self):
    self.robot_reset_pub_.publish()
    rospy.sleep(1)
    self.assertEqual(self.test_agent_.robot_resets_, 1)
  
  def test_robot_restart(self):
    self.robot_restart_pub_.publish()
    rospy.sleep(1)
    self.assertEqual(self.test_agent_.robot_restarts_, 1)
  
  def test_robot_started(self):
    goal = RobotStartedGoal()
    self.robot_started_ac_.send_goal(goal)
    self.robot_started_ac_.wait_for_result()
    self.assertTrue(self.test_agent_.exploration_)
    self.assertTrue(self.test_passed_)
  
  def test_start_exploration(self):
    self.test_agent_.start_exploration(robotModeMsg.MODE_DEEP_EXPLORATION, False)
    self.assertEqual(self.test_agent_.current_exploration_mode_,
                      robotModeMsg.MODE_DEEP_EXPLORATION)
  
  def test_start_robot(self):
    self.test_agent_.start_robot()
    rospy.sleep(1)
    self.assertTrue(self.test_passed_)
  
  def test_state_changer(self):
    self.test_agent_.change_robot_state(robotModeMsg.MODE_IDENTIFICATION)
    rospy.sleep(1)
    self.assertTrue(self.test_passed_)
  
  def test_teleoperation(self):
    msg = robotModeMsg()
    msg.nodeName = 'test_agent'
    msg.mode = msg.MODE_TELEOPERATED_LOCOMOTION
    msg.type = msg.TYPE_TRANSITION
    self.teleoperation_pub_.publish(msg)
    rospy.sleep(1)
    self.assertTrue(self.test_agent_.teleoperation_)
  
  def test_teleoperation_ended(self):
    self.test_agent_.teleoperation_ = True
    self.teleoperation_ended_pub_.publish()
    rospy.sleep(1)
    self.assertFalse(self.test_agent_.teleoperation_)
  
  def test_valid_victims(self):
    msg = Int32(data = 2)
    self.valid_victims_pub_.publish(msg)
    rospy.sleep(1)
    self.assertEqual(self.test_agent_.valid_victims_, 2)
  
  def test_validate_victim_ended(self):
    goal = ValidateVictimEndedGoal()
    self.validate_victim_ended_ac_.send_goal(goal)
    self.validate_victim_ended_ac_.wait_for_result()
    self.assertTrue(self.test_passed_)

if __name__ == '__main__':
  rospy.sleep(1)
  rospy.init_node('test_agent', anonymous=True)
  rostest.rosrun('pandora_fsm', 'test_agent', TestAgent, sys.argv)
