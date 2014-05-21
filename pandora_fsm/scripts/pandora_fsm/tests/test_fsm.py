#!/usr/bin/env python

import roslib; roslib.load_manifest('pandora_fsm')
import rospy
import unittest
import threading

import pandora_fsm.stubs.monitor_start_stub
import pandora_fsm.stubs.monitor_victim_stub
import pandora_fsm.stubs.monitor_victim_update_stub
import pandora_fsm.stubs.victim_verification_stub
import communications
import global_vars
import agent_unit_tests

from actionlib import *
from actionlib.msg import *

from state_manager_communications.msg import robotModeMsg
from pandora_fsm.agent.agent_servers import *
from pandora_fsm.agent.agent_communications import *
from data_fusion_communications.msg import QrNotificationMsg, VictimFoundMsg, \
                                            VictimToFsmMsg
from std_msgs.msg import Int32

class TestFSM(unittest.TestCase):
  
  def test_robot_start_state(self):
    rospy.sleep(3.)
    
    msg = robotModeMsg()
    msg.type = msg.TYPE_TRANSITION
    msg.mode = msg.MODE_START_AUTONOMOUS
    global_vars.com.monitor_start_pub_.publish(msg)
    
    rospy.sleep(25.)
    self.assertTrue(global_vars.test_agent.exploration_)
  
  def test_exploration_state(self):
    rospy.sleep(20.)
    
    global_vars.com.add_victims(2)
    msg = VictimFoundMsg(victimNotificationType = VictimFoundMsg.TYPE_CAMERA)
    global_vars.com.monitor_victim_pub_.publish(msg)
    
    rospy.sleep(4.)
    self.assertFalse(global_vars.test_agent.exploration_)
  
  def test_victim_monitoring_state(self):
    rospy.sleep(2.)
    global_vars.com.monitor_victim_update_pub_.publish()
    rospy.sleep(15.)
    
    msg = VictimToFsmMsg()
    msg.x = 4
    msg.y = 5
    msg.probability = 0.5
    msg.sensors = ['1', '5']
    global_vars.com.victim_verification_pub_.publish(msg)
    
    rospy.sleep(1.)
    self.assertTrue(global_vars.com.test_passed_)
    global_vars.com.test_passed_ = False
  
  def test_victim_validation_state(self):
    rospy.sleep(14.)
    self.assertTrue(global_vars.com.test_passed_)
    global_vars.com.test_passed_ = False
  
  def test_exploration_state_change_to_normal(self):
    rospy.sleep(10.)
    global_vars.test_agent.initial_time_ = rospy.get_rostime().secs - 600
    msg = QrNotificationMsg()
    for i in range(5):
      global_vars.com.qr_notification_pub_.publish(msg)
      rospy.Rate(2).sleep()
    msg = Int32(data = 1)
    global_vars.com.valid_victims_pub_.publish(msg)
    rospy.Rate(2).sleep()
    global_vars.com.robot_restart_pub_.publish()
    rospy.sleep(2.)
    self.assertEqual(global_vars.test_agent.current_exploration_mode_,
                      ExplorationModeGoal.MODE_NORMAL)
  
  def test_reset(self):
    rospy.sleep(15.)
    global_vars.com.robot_reset_pub_.publish()
    
    rospy.sleep(4.)
    msg = robotModeMsg()
    msg.type = msg.TYPE_TRANSITION
    msg.mode = msg.MODE_START_AUTONOMOUS
    global_vars.com.monitor_start_pub_.publish(msg)
    
    rospy.sleep(25.)
    self.assertTrue(global_vars.test_agent.exploration_)
    self.assertEqual(global_vars.test_agent.robot_resets_, 1)
  
  def test_move_to_orange_without_victims_found(self):
    msg = ArenaTypeMsg(arenaType = ArenaTypeMsg.TYPE_ORANGE)
    global_vars.com.arena_type_pub_.publish(msg)
    rospy.Rate(2).sleep()
    self.assertTrue(global_vars.test_agent.turn_back_)
    rospy.sleep(8.)
    global_vars.com.add_victims(1)
    msg = VictimFoundMsg(victimNotificationType = VictimFoundMsg.TYPE_CAMERA)
    global_vars.com.monitor_victim_pub_.publish(msg)
    rospy.sleep(20.)
    msg = VictimToFsmMsg()
    msg.x = 4
    msg.y = 5
    msg.probability = 0.5
    msg.sensors = ['1', '5']
    global_vars.com.victim_verification_pub_.publish(msg)
    rospy.Rate(2).sleep()
    msg = Int32(data = 1)
    global_vars.com.valid_victims_pub_.publish(msg)
    rospy.sleep(15.)
    msg = ArenaTypeMsg(arenaType = ArenaTypeMsg.TYPE_ORANGE)
    global_vars.com.arena_type_pub_.publish(msg)
    rospy.sleep(5.)
    self.assertEqual(global_vars.test_agent.current_arena_, ArenaTypeMsg.TYPE_ORANGE)
    self.assertTrue(global_vars.test_agent.teleoperation_)

if __name__ == '__main__':
  rospy.init_node('test_fsm', anonymous=True)
  global_vars.init(False)
  
  #start the agent in a new thread
  agent_thread = threading.Thread(target = global_vars.test_agent.main)
  agent_thread.start()
  
  #test ROBOT_START state
  robot_start_state = unittest.TestSuite()
  robot_start_state.addTest(TestFSM('test_robot_start_state'))
  unittest.TextTestRunner(verbosity=1).run(robot_start_state)
  
  #test EXPLORATION state
  exploration_state = unittest.TestSuite()
  exploration_state.addTest(TestFSM('test_exploration_state'))
  unittest.TextTestRunner(verbosity=1).run(exploration_state)
  
  #test VICTIM_MONITORING state
  #update the victim and then move to it
  victim_monitoring_state = unittest.TestSuite()
  victim_monitoring_state.addTest(TestFSM('test_victim_monitoring_state'))
  unittest.TextTestRunner(verbosity=1).run(victim_monitoring_state)
  
  #test VICTIM_VALIDATION state
  victim_validation_state = unittest.TestSuite()
  victim_validation_state.addTest(TestFSM('test_victim_validation_state'))
  unittest.TextTestRunner(verbosity=1).run(victim_validation_state)
  
  #change EXPLORATION mode to MODE_EXPLORATION
  exploration_normal = unittest.TestSuite()
  exploration_normal.addTest(TestFSM('test_exploration_state_change_to_normal'))
  unittest.TextTestRunner(verbosity=1).run(exploration_normal)
  
  #reset the robot
  reset_robot = unittest.TestSuite()
  reset_robot.addTest(TestFSM('test_reset'))
  unittest.TextTestRunner(verbosity=1).run(reset_robot)
  
  #test move to orange without victims found, find the first victim
  #and get back to orange
  move_to_orange = unittest.TestSuite()
  move_to_orange.addTest(TestFSM('test_move_to_orange_without_victims_found'))
  unittest.TextTestRunner(verbosity=1).run(move_to_orange)
  
  global_vars.com.delete_action_servers()
