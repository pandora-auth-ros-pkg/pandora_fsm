#!/usr/bin/env python

import roslib; roslib.load_manifest('pandora_fsm')
import rospy
import pandora_fsm
import threading

from state_manager_communications.msg import RobotModeAction, RobotModeGoal, \
                                              robotModeMsg
from std_msgs.msg import Int32, Empty
from fsm_communications.msg import *
from pandora_data_fusion_msgs.msg import QrNotificationMsg, VictimsMsg, \
                                          ValidateVictimAction, \
                                          ValidateVictimGoal
from math import exp, log
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from pandora_navigation_msgs.msg import InitialTurnAction, InitialTurnGoal
from target_selector_communications.msg import SelectTargetAction, \
                                                SelectTargetGoal

from dynamic_reconfigure.server import Server
from pandora_fsm.cfg import FSMParamsConfig

from actionlib import *
from actionlib.msg import *

state_changer_action_topic = '/robot/state/change'
robot_turn_back_topic = 'robot_turn_back'
exploration_mode_topic = 'exploration_mode'
arena_type_topic = '/arena_type'
robocup_score_topic = '/data_fusion/alert_handler/robocup_score'
qr_notification_topic = '/data_fusion/alert_handler/qr_notification'
delete_victim_topic = '/data_fusion/alert_handler/delete_victim'
data_fusion_validate_victim_topic = '/data_fusion/alert_handler/validate_victim'
gui_validation_topic = '/gui/validate_victim'
victims_topic = '/data_fusion/alert_handler/victims'
state_monitor_topic = '/robot/state/clients'
teleoperation_ended_topic = '/teleoperation_ended'
robot_reset_topic = '/robot_reset'
robot_restart_topic = '/robot_restart'
start_button_topic = '/start_button'
initial_turn_topic = '/initial_turn'
move_base_topic = '/move_base'
select_target_topic = '/select_target'

class AgentCommunications():
  
  def __init__(self):
    self.current_arena_ = ArenaTypeMsg.TYPE_YELLOW
    self.current_score_ = 0
    self.valid_victims_ = 0
    self.qrs_ = 0
    self.current_exploration_mode_ = 0
    self.previous_exploration_mode_ = ExplorationModeGoal.MODE_DEEP
    self.current_victims_ = []
    
    self.robot_resets_ = 0
    self.robot_restarts_ = 0
    
    self.strategy_ = EmptyState()
    self.start_button_strategy_ = 0
    self.validate_victim_strategy_ = 1
    self.new_fsm_strategy_ = ChangeExplorationModeState()
    self.select_cost_function_ = 0
    
    Server(FSMParamsConfig, self.reconfigure)
    
    rospy.Subscriber(arena_type_topic, ArenaTypeMsg, self.arena_type_cb)
    rospy.Subscriber(robocup_score_topic, Int32, self.score_cb)
    rospy.Subscriber(qr_notification_topic, QrNotificationMsg,
                      self.qr_notification_cb)
    rospy.Subscriber(state_monitor_topic, robotModeMsg, self.state_monitor_cb)
    rospy.Subscriber(teleoperation_ended_topic, Empty,
                      self.teleoperation_ended_cb)
    rospy.Subscriber(robot_reset_topic, Empty, self.robot_reset_cb)
    rospy.Subscriber(robot_restart_topic, Empty, self.robot_restart_cb)
    rospy.Subscriber(start_button_topic, Empty, self.start_button_cb)
    rospy.Subscriber(victims_topic, VictimsMsg, self.victims_cb)
    
    self.state_changer_ac_ = SimpleActionClient(state_changer_action_topic,
                                                RobotModeAction)
    #~ self.state_changer_ac_.wait_for_server()
    self.robot_turn_back_ac_ = SimpleActionClient(robot_turn_back_topic,
                                                  RobotTurnBackAction)
    #~ self.robot_turn_back_ac_.wait_for_server()
    self.exploration_mode_ac_ = SimpleActionClient(exploration_mode_topic,
                                                    ExplorationModeAction)
    #~ self.exploration_mode_ac_.wait_for_server()
    self.select_target_ac_ = SimpleActionClient(select_target_topic,
                                                SelectTargetAction)
    #~ self.select_target_ac_.wait_for_server()
    self.move_base_ac_ = SimpleActionClient(move_base_topic, MoveBaseAction)
    #~ self.move_base_ac_.wait_for_server()
    self.delete_victim_ac_ = SimpleActionClient(delete_victim_topic,
                                                DeleteVictimAction)
    #~ self.delete_victim_ac_.wait_for_server()
    self.gui_validate_victim_ac_ = SimpleActionClient(gui_validation_topic,
                                                      ValidateVictimGUIAction)
    #~ self.gui_validate_victim_ac_.wait_for_server()
    self.initial_turn_ac_ = SimpleActionClient(initial_turn_topic,
                                                InitialTurnAction)
    #~ self.initial_turn_ac_.wait_for_server()
    self.data_fusion_validate_victim_ac_ = \
      SimpleActionClient(data_fusion_validate_victim_topic,
                          ValidateVictimAction)
    #~ self.data_fusion_validate_victim_ac_.wait_for_server()
  
  def main(self):
    while not rospy.is_shutdown():
      self.strategy_.main()
      rospy.loginfo('agent loop')
      rospy.Rate(1).sleep()
  
  def start_button_cb(self, msg):
    self.strategy_ = StartButtonState()
  
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
  
  def state_monitor_cb(self, msg):
    rospy.loginfo('teleoperation_cb')
    if msg.type == msg.TYPE_TRANSITION and \
        msg.mode == robotModeMsg.MODE_TELEOPERATED_LOCOMOTION:
      self.strategy_ = EmptyState()
      self.end_exploration()
    elif msg.type == msg.TYPE_TRANSITION and \
        msg.mode == robotModeMsg.MODE_START_AUTONOMOUS:
      if self.strategy_ == StartButtonState():
        self.strategy_ = RobotStartState()
  
  def teleoperation_ended_cb(self, msg):
    rospy.loginfo('teleoperation_ended_cb')
    self.strategy_ = 2
  
  def robot_reset_cb(self, msg):
    rospy.loginfo('robot_reset_cb')
    
    self.end_exploration()
    rospy.sleep(1.)
    
    self.current_arena_ = ArenaTypeMsg.TYPE_YELLOW
    self.current_score_ = 0
    self.valid_victims_ = 0
    self.qrs_ = 0
    self.previous_exploration_mode_ = ExplorationModeGoal.MODE_DEEP
    self.current_victims_ = []
    
    self.strategy_ = EmptyState()
    self.start_button_strategy_ = 0
    self.validate_victim_strategy_ = 1
    
    self.robot_resets_ += 1
  
  def robot_restart_cb(self, msg):
    rospy.loginfo('robot_restart_cb')
    self.robot_restarts_ += 1
  
  def score_cb(self, msg):
    rospy.loginfo('score_cb')
    self.current_score_ = msg.data
  
  def victims_cb(self, msg):
    rospy.loginfo('victims_cb')
    
    if self.strategy_ == ExplorationState() or \
        self.strategy_ == ChangeExplorationModeState():
      self.current_victims_ = msg.victims
      self.strategy_ = CheckVictimListState()
    elif self.strategy_ == IdentificationState():
      self.current_victims_ = msg.victims
      self.strategy_ = UpdateVictimState()
  
  def change_robot_state(self, new_state):
    rospy.loginfo('change_robot_state')
    mode_msg = robotModeMsg(nodeName = 'agent', mode = new_state)
    mode_goal = RobotModeGoal(modeMsg = mode_msg)
    
    self.state_changer_ac_.send_goal(mode_goal)
    self.state_changer_ac_.wait_for_result()
  
  def end_exploration(self):
    rospy.loginfo('end_exploration')
    
    self.previous_exploration_mode_ = self.current_exploration_mode_
    self.current_exploration_mode_ = 0
    self.select_target_ac_.cancel_all_goals()
    self.move_base_ac_.cancel_all_goals()
  
  def start_exploration(self, exploration_mode):
    rospy.loginfo('start_exploration = %i' % exploration_mode)
    
    if self.current_exploration_mode_ != 0:
      self.end_exploration()
    else:
      self.change_robot_state(robotModeMsg.MODE_EXPLORATION)
    
    rospy.Rate(2).sleep()
    self.current_exploration_mode_ = exploration_mode
    goal = ExplorationModeGoal(explorationMode = exploration_mode)
    self.exploration_mode_ac_.send_goal(goal)
    self.exploration_mode_ac_.wait_for_result()
  
  def reconfigure(self, config, level):
    self.max_time_ = config["maxTime"]
    self.max_victims_ = config["arenaVictims"]
    self.max_qrs_ = config["maxQRs"]
    self.initial_time_ = rospy.get_rostime().secs - config["timePassed"]
    if not config["newFSMStrategy"]:
      self.new_fsm_strategy_ = EmptyState()
    self.select_cost_function_ = config["explorationCostFunction"]
    return config

class EmptyState(AgentCommunications):
  
  def main(self):
    pass

class StartButtonState(AgentCommunications):
  
  def main(self):
    if self.start_button_strategy_ == 1:
      pass

class RobotStartState(AgentCommunications):
  
  def main(self):
    self.wait_for_slam()
    self.initial_turn()
    self.strategy_ = ExplorationState()
  
  def wait_for_slam(self):
    rospy.loginfo('wait_for_slam')
    rospy.sleep(10.)
  
  def initial_turn(self):
    rospy.loginfo('initial_turn')
    goal = InitialTurnGoal()
    self.initial_turn_ac_.send_goal(goal)
    self.initial_turn_ac_.wait_for_result()

class ExplorationState(AgentCommunications):
  
  def main(self):
    self.strategy_ = self.new_fsm_strategy_
    exploration_thread = threading.Thread(target = self.exploration)
    exploration_thread.start()
  
  def exploration(self):
    if self.current_exploration_mode_ == 0:
      self.start_exploration(self.previous_exploration_mode_)
    target = self.select_target()
    if target is not None:
      self.move_to_target(target)
  
  def select_target(self):
    rospy.loginfo('select_target')
    goal = SelectTargetGoal(targetType = TYPE_EXPLORATION)
    
    state = actionlib.GoalStatus.ABORTED
    
    while state != actionlib.GoalStatus.SUCCEEDED:
      self.select_target_ac_.send_goal(goal)
      self.select_target_ac_.wait_for_result()
      state = self.select_target_ac_.get_state()
      if state == actionlib.GoalStatus.PREEMPTED:
        return None
    
    return self.select_target_ac_.get_result()
  
  def move_to_target(self, target):
    rospy.loginfo('move_to_target')
    goal = MoveBaseGoal(target_pose = target)
    self.move_base_ac_.send_goal(goal)
    self.move_base_ac_.wait_for_result()
    self.strategy_ = ExplorationState()

class ChangeExplorationModeState(AgentCommunications):
  
  def __init__(self):
    if self.select_cost_function_ == 0:
      self.cost_function = self.cost_function1
    elif self.select_cost_function_ == 1:
      self.cost_function = self.cost_function2
  
  def main(self):
    self.evaluate_current_situation(self.current_arena_)
  
  def cost_function1(self):
    time = float(rospy.get_rostime().secs - self.initial_time_)
    
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
    rospy.loginfo('evaluate_current_situation')
    if arena_type == ArenaTypeMsg.TYPE_YELLOW:
      current_cost = self.cost_function()
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
        self.validate_victim_strategy_ = 2
      else:
        self.current_arena_ = ArenaTypeMsg.TYPE_ORANGE
        self.strategy_ = 0
        self.abort_fsm_pub_.publish()
    elif arena_type == ArenaTypeMsg.TYPE_YELLOW_BLACK:
      rospy.Rate(1).sleep()

class CheckVictimListState(AgentCommunications):
  
  def main(self):
    self.check_new_victims()
  
  def check_new_victims(self):
    rospy.loginfo('check_new_victims')
    max_victim = self.find_max_probability_victim(self.current_victims_)
    if max_victim[1].probability > 0.5:
      self.target_victim_ = max_victim
      
      self.end_exploration()
      
      self.change_robot_state(robotModeMsg.MODE_IDENTIFICATION)
      self.strategy_ = IdentificationState()
    else:
      self.strategy_ = ChangeExplorationModeState()
  
  def find_max_probability_victim(self, victim_list):
    rospy.loginfo('find_max_probability_victim')
    
    max_victim = victim_list[0]
    
    for i in range(1, len(victim_list)):
      if victim_list[i].probability > max_victim.probability:
        max_victim = victim_list[i]
    
    return max_victim

class UpdateVictimState(AgentCommunications):
  
  def main(self):
    self.update_victim()
  
  def update_victim(self):
    rospy.loginfo('update_victim')
    
    for victim in self.current_victims_:
      if victim.id == self.target_victim_.id:
        if self.target_victim_.probability != \
            self.current_victims_[victim_id].probability:
          self.move_base_ac_.cancel_goal()
          self.target_victim_ = victim
        self.strategy_ = IdentificationState()
        break
    else:
      self.move_base_ac_.cancel_goal()
      self.identification_strategy_ = 1
      self.strategy_ = ExplorationState()

class IdentificationState(AgentCommunications):
  
  def main(self):
    self.strategy_ = EmptyState()
    move_to_victim_thread = threading.Thread(target = self.move_to_victim)
    move_to_victim_thread.start()
  
  def move_to_victim(self):
    rospy.loginfo('move_to_victim')
    
    goal = MoveBaseGoal(target_pose = self.target_victim_[1].victimPose)
    self.move_base_ac_.send_goal(goal)
    self.move_base_ac_.wait_for_result()
    
    if self.move_base_ac_.get_state() == actionlib.GoalStatus.SUCCEEDED:
      self.strategy_ = DataFusionHoldState()
    elif self.move_base_ac_.get_state() == actionlib.GoalStatus.ABORTED:
      goal = DeleteVictimGoal(self.target_victim_.id)
      self.delete_victim_ac_.send_goal(goal)
      self.delete_victim_ac_.wait_for_result()
      self.strategy_ = CheckVictimListState()

class DataFusionHoldState(self):
  
  def main(self):
    self.data_fusion_hold()
  
  def data_fusion_hold(self):
    rospy.loginfo('data_fusion_hold')
    self.change_robot_state(robotModeMsg.MODE_DF_HOLD)
    rospy.sleep(10.)
    
    for victim in self.current_victims_:
      if victim.id == self.target_victim_.id:
        if self.current_victims_[victim.id].probability > 0.8:
          self.strategy_ = ValidationState()
        else:
          self.strategy_ = CheckVictimListState()
        break

class ValidationState(AgentCommunications):
  
  def main(self):
    if self.validate_victim_strategy_ == 1:
      self.validate_current_victim()
    elif self.validate_victim_strategy_ == 2:
      self.validate_current_victim()
      self.enable_frontier()
  
  def enable_frontier(self):
    rospy.loginfo('enable_frontier')
    goal = RobotTurnBackGoal(frontierExploration = True)
    self.robot_turn_back_ac_.send_goal(goal)
    self.robot_turn_back_ac_.wait_for_result()
    self.validate_victim_strategy_ = 1
  
  def validate_current_victim(self):
    rospy.loginfo('validate_current_victim')
    goal = ValidateVictimGUIGoal();
    goal.victimFoundx = target_victim_.victimPose.pose.position.x
    goal.victimFoundy = target_victim_.victimPose.pose.position.y
    goal.probability = target_victim_.probability
    goal.sensorIDsFound = target_victim_.sensors
    self.gui_validate_victim_ac_.send_goal(goal)
    self.gui_validate_victim_ac_.wait_for_result()
    
    result = self.gui_validate_victim_ac_.get_result()
    
    if result.victimValid:
      self.valid_victims_ += 1
    
    goal = ValidateVictimGoal();
    goal.victimId = target_victim_.id
    goal.victimValid = result.victimValid
    self.data_fusion_validate_victim_ac_.send_goal(goal)
    self.data_fusion_validate_victim_ac_.wait_for_result()
    
    self.strategy_ = CheckVictimListState()
