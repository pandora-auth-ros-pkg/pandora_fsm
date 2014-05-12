#!/usr/bin/env python

import roslib; roslib.load_manifest('pandora_fsm')
import rospy
import unittest

import communications
import global_vars

from actionlib import *
from actionlib.msg import *

from state_manager_communications.msg import robotModeMsg
from pandora_fsm.agent.agent_servers import *
from pandora_fsm.agent.agent_communications import *
from data_fusion_communications.msg import QrNotificationMsg
from std_msgs.msg import Int32

class TestAgent(unittest.TestCase):

  @classmethod
  def tearDownClass(cls):
    rospy.sleep(5.)
    global_vars.com.robot_start_sub_.unregister()
    global_vars.com.exploration_start_sub_.unregister()
    global_vars.com.monitor_victim_start_sub_.unregister()
    global_vars.com.validate_victim_start_sub_.unregister()
    global_vars.com.abort_fsm_sub_.unregister()
    global_vars.com.state_changer_as_.__del__()
    global_vars.com.robot_turn_back_as_.__del__()
    global_vars.com.return_to_orange_as_.__del__()
  
  def setUp(self):
    global_vars.test_agent.current_arena_ = ArenaTypeMsg.TYPE_YELLOW
    global_vars.test_agent.current_score_ = 0
    global_vars.test_agent.valid_victims_ = 0
    global_vars.test_agent.qrs_ = 0
    global_vars.test_agent.current_exploration_mode_ = 0
    global_vars.test_agent.exploration_ = False
    global_vars.test_agent.teleoperation_ = False
    global_vars.test_agent.turn_back_ = False
    global_vars.test_agent.robot_resets_ = 0
    global_vars.test_agent.robot_restarts_ = 0
  
  def tearDown(self):
    global_vars.com.test_passed_ = False
  
  def test_cost_function(self):
    rospy.loginfo('test_cost_function')
    global_vars.test_agent.valid_victims_ = 2
    global_vars.test_agent.qrs_ = 10
    global_vars.test_agent.robot_resets_ = 1
    global_vars.test_agent.robot_restarts_ = 4
    cost = global_vars.test_agent.cost_function(600.0)
    self.assertEqual(cost, 64.6837664225596)
  
  def test_evaluate_current_situation_orange_teleoperation(self):
    rospy.loginfo('test_evaluate_current_situation_orange_teleoperation')
    global_vars.test_agent.valid_victims_ = 1
    global_vars.test_agent.evaluate_current_situation(ArenaTypeMsg.TYPE_ORANGE)
    rospy.Rate(2).sleep()
    self.assertTrue(global_vars.test_agent.teleoperation_)
    self.assertEqual(global_vars.test_agent.current_arena_, ArenaTypeMsg.TYPE_ORANGE)
    self.assertTrue(global_vars.com.test_passed_)
  
  def test_evaluate_current_situation_orange_turn_back(self):
    rospy.loginfo('test_evaluate_current_situation_orange_turn_back')
    global_vars.test_agent.evaluate_current_situation(ArenaTypeMsg.TYPE_ORANGE)
    self.assertTrue(global_vars.test_agent.turn_back_)
    self.assertTrue(global_vars.com.test_passed_)
  
  def test_evaluate_current_situation_yellow_deep(self):
    rospy.loginfo('test_evaluate_current_situation_yellow_deep')
    global_vars.test_agent.initial_time_ = rospy.get_rostime().secs - 600
    global_vars.test_agent.valid_victims_ = 0
    global_vars.test_agent.qrs_ = 7
    global_vars.test_agent.robot_resets_ = 0
    global_vars.test_agent.robot_restarts_ = 2
    global_vars.test_agent.evaluate_current_situation(ArenaTypeMsg.TYPE_YELLOW)
    rospy.Rate(2).sleep()
    self.assertEqual(global_vars.test_agent.current_exploration_mode_,
                      robotModeMsg.MODE_DEEP_EXPLORATION)
    self.assertTrue(global_vars.com.test_passed_)
  
  def test_evaluate_current_situation_yellow_fast(self):
    rospy.loginfo('test_evaluate_current_situation_yellow_fast')
    global_vars.test_agent.initial_time_ = rospy.get_rostime().secs - 600
    global_vars.test_agent.valid_victims_ = 2
    global_vars.test_agent.qrs_ = 10
    global_vars.test_agent.robot_resets_ = 1
    global_vars.test_agent.robot_restarts_ = 4
    global_vars.test_agent.evaluate_current_situation(ArenaTypeMsg.TYPE_YELLOW)
    rospy.Rate(2).sleep()
    self.assertEqual(global_vars.test_agent.current_exploration_mode_,
                      robotModeMsg.MODE_FAST_EXPLORATION)
    self.assertTrue(global_vars.com.test_passed_)
  
  def test_evaluate_current_situation_yellow_normal(self):
    rospy.loginfo('test_evaluate_current_situation_yellow_normal')
    global_vars.test_agent.initial_time_ = rospy.get_rostime().secs - 600
    global_vars.test_agent.valid_victims_ = 1
    global_vars.test_agent.qrs_ = 6
    global_vars.test_agent.robot_resets_ = 0
    global_vars.test_agent.robot_restarts_ = 1
    global_vars.test_agent.evaluate_current_situation(ArenaTypeMsg.TYPE_YELLOW)
    rospy.Rate(2).sleep()
    self.assertEqual(global_vars.test_agent.current_exploration_mode_,
                      robotModeMsg.MODE_EXPLORATION)
    self.assertTrue(global_vars.com.test_passed_)
  
  def test_exploration_ended(self):
    rospy.loginfo('test_exploration_ended')
    global_vars.test_agent.exploration_ = True
    goal = ExplorationEndedGoal()
    global_vars.com.exploration_ended_ac_.send_goal(goal)
    global_vars.com.exploration_ended_ac_.wait_for_result()
    rospy.Rate(2).sleep()
    self.assertFalse(global_vars.test_agent.exploration_)
    self.assertTrue(global_vars.com.test_passed_)
  
  def test_exploration_restart(self):
    rospy.loginfo('test_exploration_restart')
    goal = ExplorationRestartGoal()
    global_vars.com.exploration_restart_ac_.send_goal(goal)
    global_vars.com.exploration_restart_ac_.wait_for_result()
    self.assertTrue(global_vars.test_agent.exploration_)
  
  def test_monitor_victim_ended(self):
    rospy.loginfo('test_monitor_victim_ended')
    goal = MonitorVictimEndedGoal()
    global_vars.com.monitor_victim_ended_ac_.send_goal(goal)
    global_vars.com.monitor_victim_ended_ac_.wait_for_result()
    rospy.Rate(2).sleep()
    self.assertTrue(global_vars.com.test_passed_)
  
  def test_qr_notification(self):
    rospy.loginfo('test_qr_notification')
    msg = QrNotificationMsg()
    global_vars.com.qr_notification_pub_.publish(msg)
    rospy.Rate(2).sleep()
    self.assertEqual(global_vars.test_agent.qrs_, 1)
  
  def test_robocup_score(self):
    rospy.loginfo('test_robocup_score')
    msg = Int32(data = 25)
    global_vars.com.robocup_score_pub_.publish(msg)
    rospy.Rate(2).sleep()
    self.assertEqual(global_vars.test_agent.current_score_, 25)
  
  def test_robot_reset(self):
    rospy.loginfo('test_robot_reset')
    global_vars.com.robot_reset_pub_.publish()
    rospy.Rate(2).sleep()
    self.assertEqual(global_vars.test_agent.robot_resets_, 1)
  
  def test_robot_restart(self):
    rospy.loginfo('test_robot_restart')
    global_vars.com.robot_restart_pub_.publish()
    rospy.Rate(2).sleep()
    self.assertEqual(global_vars.test_agent.robot_restarts_, 1)
  
  def test_robot_started(self):
    rospy.loginfo('test_robot_started')
    goal = RobotStartedGoal()
    global_vars.com.robot_started_ac_.send_goal(goal)
    global_vars.com.robot_started_ac_.wait_for_result()
    rospy.Rate(2).sleep()
    self.assertTrue(global_vars.test_agent.exploration_)
    self.assertEqual(global_vars.test_agent.current_exploration_mode_,
                      robotModeMsg.MODE_DEEP_EXPLORATION)
    self.assertTrue(global_vars.com.test_passed_)
  
  def test_start_exploration(self):
    rospy.loginfo('test_start_exploration')
    global_vars.test_agent.start_exploration(robotModeMsg.MODE_DEEP_EXPLORATION, False)
    rospy.Rate(2).sleep()
    self.assertEqual(global_vars.test_agent.current_exploration_mode_,
                      robotModeMsg.MODE_DEEP_EXPLORATION)
    self.assertTrue(global_vars.com.test_passed_)
  
  def test_start_robot(self):
    rospy.loginfo('test_start_robot')
    global_vars.test_agent.start_robot()
    rospy.Rate(2).sleep()
    self.assertTrue(global_vars.com.test_passed_)
  
  def test_state_changer(self):
    rospy.loginfo('test_state_changer')
    global_vars.test_agent.change_robot_state(robotModeMsg.MODE_IDENTIFICATION)
    self.assertTrue(global_vars.com.test_passed_)
  
  def test_teleoperation(self):
    rospy.loginfo('test_teleoperation')
    msg = robotModeMsg()
    msg.nodeName = 'global_vars.test_agent'
    msg.mode = msg.MODE_TELEOPERATED_LOCOMOTION
    msg.type = msg.TYPE_TRANSITION
    global_vars.com.teleoperation_pub_.publish(msg)
    rospy.Rate(2).sleep()
    self.assertTrue(global_vars.test_agent.teleoperation_)
    self.assertTrue(global_vars.com.test_passed_)
  
  def test_teleoperation_ended(self):
    rospy.loginfo('test_teleoperation_ended')
    global_vars.test_agent.teleoperation_ = True
    global_vars.com.teleoperation_ended_pub_.publish()
    rospy.Rate(2).sleep()
    self.assertFalse(global_vars.test_agent.teleoperation_)
  
  def test_valid_victims(self):
    rospy.loginfo('test_valid_victims')
    msg = Int32(data = 2)
    global_vars.com.valid_victims_pub_.publish(msg)
    rospy.Rate(2).sleep()
    self.assertEqual(global_vars.test_agent.valid_victims_, 2)
  
  def test_validate_victim_ended(self):
    rospy.loginfo('test_validate_victim_ended')
    goal = ValidateVictimEndedGoal()
    global_vars.com.validate_victim_ended_ac_.send_goal(goal)
    global_vars.com.validate_victim_ended_ac_.wait_for_result()
    rospy.Rate(2).sleep()
    self.assertTrue(global_vars.com.test_passed_)
  
  def test_validate_victim_ended_return_to_orange(self):
    rospy.loginfo('test_validate_victim_ended_return_to_orange')
    global_vars.test_agent.turn_back_ = True
    goal = ValidateVictimEndedGoal()
    global_vars.com.validate_victim_ended_ac_.send_goal(goal)
    global_vars.com.validate_victim_ended_ac_.wait_for_result()
    rospy.Rate(2).sleep()
    self.assertFalse(global_vars.test_agent.turn_back_)
    self.assertTrue(global_vars.com.test_passed_)

if __name__ == '__main__':
  rospy.init_node('test_agent', anonymous=True)
  global_vars.init(True)
  suite = unittest.TestLoader().loadTestsFromTestCase(TestAgent)
  unittest.TextTestRunner(verbosity=1).run(suite)
