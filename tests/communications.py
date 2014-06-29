#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2014, P.A.N.D.O.R.A. Team.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of P.A.N.D.O.R.A. Team nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Voulgarakis George <turbolapingr@gmail.com>

import roslib
roslib.load_manifest('pandora_fsm')
import rospy
import dynamic_reconfigure.client
import global_vars

from actionlib import SimpleActionServer, SimpleActionClient
from pandora_fsm.robocup_agent import agent_topics

from state_manager_communications.msg import robotModeMsg, RobotModeAction
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32, Float32
from pandora_end_effector_planner.msg import MoveEndEffectorAction, \
    MoveEndEffectorResult
from pandora_rqt_gui.msg import ValidateVictimGUIAction, ValidateVictimGUIResult
from pandora_data_fusion_msgs.msg import WorldModelMsg, VictimInfoMsg, \
    QrNotificationMsg, ValidateVictimAction, ValidateVictimResult, \
    DeleteVictimAction, DeleteVictimResult
from move_base_msgs.msg import MoveBaseAction, MoveBaseFeedback, MoveBaseResult
from pandora_navigation_msgs.msg import ArenaTypeMsg, DoExplorationAction, \
    DoExplorationFeedback, DoExplorationResult


class Communications():

    def __init__(self, high_logic_type):

        self.high_logic_type = high_logic_type
        self.robot_pose_ = PoseStamped()
        self.move_base_succeeded_ = False
        self.move_base_aborted_ = False
        self.move_end_effector_succeeded_ = False
        self.move_end_effector_aborted_ = False

        if high_logic_type == 'agent':
            self.dynamic_reconfigure_client = \
                dynamic_reconfigure.client.Client("test_agent")
        else:
            self.state_changer_ac_ = \
                SimpleActionClient(agent_topics.state_changer_action_topic,
                                   RobotModeAction)

        self.arena_type_pub_ = rospy.Publisher(agent_topics.arena_type_topic,
                                               ArenaTypeMsg)
        self.robocup_score_pub_ = \
            rospy.Publisher(agent_topics.robocup_score_topic, Int32)
        self.qr_notification_pub_ = \
            rospy.Publisher(agent_topics.qr_notification_topic,
                            QrNotificationMsg)
        self.area_covered_pub_ = \
            rospy.Publisher(agent_topics.area_covered_topic, Float32)
        self.victims_pub_ = rospy.Publisher(agent_topics.world_model_topic,
                                            WorldModelMsg)
        self.do_exploration_as_ = \
            SimpleActionServer(agent_topics.do_exploration_topic,
                               DoExplorationAction,
                               self.do_exploration_cb,
                               auto_start=False)
        self.do_exploration_as_.start()

        self.move_base_as_ = SimpleActionServer(agent_topics.move_base_topic,
                                                MoveBaseAction,
                                                self.move_base_cb,
                                                auto_start=False)
        self.move_base_as_.start()

        self.delete_victim_as_ = \
            SimpleActionServer(agent_topics.delete_victim_topic,
                               DeleteVictimAction,
                               self.delete_victim_cb,
                               auto_start=False)
        self.delete_victim_as_.start()

        self.gui_validate_victim_as_ = \
            SimpleActionServer(agent_topics.gui_validation_topic,
                               ValidateVictimGUIAction,
                               self.gui_validate_victim_cb,
                               auto_start=False)
        self.gui_validate_victim_as_.start()

        self.data_fusion_validate_victim_as_ = \
            SimpleActionServer(agent_topics.data_fusion_validate_victim_topic,
                               ValidateVictimAction,
                               self.data_fusion_validate_victim_cb,
                               auto_start=False)
        self.data_fusion_validate_victim_as_.start()

        self.end_effector_planner_as_ = \
            SimpleActionServer(agent_topics.move_end_effector_planner_topic,
                               MoveEndEffectorAction,
                               self.end_effector_planner_cb,
                               auto_start=False)
        self.end_effector_planner_as_.start()

    def do_exploration_cb(self, goal):
        rospy.loginfo('do_exploration_cb')

        for i in range(10):
            rospy.Rate(1).sleep()
            self.robot_pose_.pose.position.x += 0.1
            self.robot_pose_.pose.position.y += 0.05
            feedback = DoExplorationFeedback()
            feedback.base_position.pose.position.x = \
                self.robot_pose_.pose.position.x
            feedback.base_position.pose.position.y = \
                self.robot_pose_.pose.position.y
            self.do_exploration_as_.publish_feedback(feedback)
            if self.do_exploration_as_.is_preempt_requested():
                self.do_exploration_as_.set_preempted()
                break
        else:
            result = DoExplorationResult()
            self.do_exploration_as_.set_aborted(result)

    def move_base_cb(self, goal):
        rospy.loginfo('move_base_cb')

        for i in range(10):
            rospy.Rate(1).sleep()
            self.robot_pose_.pose.position.x += 0.1
            self.robot_pose_.pose.position.y += 0.05
            feedback = MoveBaseFeedback()
            feedback.base_position.pose.position.x = \
                self.robot_pose_.pose.position.x
            feedback.base_position.pose.position.y = \
                self.robot_pose_.pose.position.y
            self.move_base_as_.publish_feedback(feedback)
            if self.move_base_succeeded_:
                self.move_base_succeeded_ = False
                result = MoveBaseResult()
                self.move_base_as_.set_succeeded(result)
                break
            if self.move_base_aborted_:
                self.move_base_aborted_ = False
                result = MoveBaseResult()
                self.move_base_as_.set_aborted(result)
                break
            if self.move_base_as_.is_preempt_requested():
                self.move_base_as_.set_preempted()
                break
        else:
            result = MoveBaseResult()
            self.move_base_as_.set_succeeded(result)

    def delete_victim_cb(self, goal):
        rospy.loginfo('delete_victim_cb')
        if self.high_logic_type == 'agent':
            for victim in global_vars.test_agent.new_victims_:
                if victim.id == goal.victimId:
                    global_vars.test_agent.new_victims_.remove(victim)

        result = DeleteVictimResult()
        self.delete_victim_as_.set_succeeded(result)

    def gui_validate_victim_cb(self, goal):
        rospy.loginfo('gui_validate_victim_cb')

        for i in range(50):
            rospy.Rate(10).sleep()

            if self.gui_validate_victim_as_.is_preempt_requested():
                self.gui_validate_victim_as_.set_preempted()
                break
        else:
            result = ValidateVictimGUIResult(victimValid=True)
            self.gui_validate_victim_as_.set_succeeded(result)

    def data_fusion_validate_victim_cb(self, goal):
        rospy.loginfo('data_fusion_validate_victim_cb')
        if self.high_logic_type == 'agent':
            for victim in global_vars.test_agent.new_victims_:
                if victim.id == goal.victimId:
                    global_vars.test_agent.new_victims_.remove(victim)

        result = ValidateVictimResult()
        self.data_fusion_validate_victim_as_.set_succeeded(result)

    def end_effector_planner_cb(self, goal):
        rospy.loginfo('end_effector_planner_cb')

        for i in range(5):
            rospy.Rate(1).sleep()
            if self.move_end_effector_succeeded_:
                self.move_end_effector_succeeded_ = False
                result = MoveEndEffectorResult()
                self.end_effector_planner_as_.set_succeeded(result)
                break
            if self.move_end_effector_aborted_:
                self.move_end_effector_aborted_ = False
                result = MoveEndEffectorResult()
                self.end_effector_planner_as_.set_aborted(result)
                break
            if self.move_base_as_.is_preempt_requested():
                self.end_effector_planner_as_.set_preempted()
                break
        else:
            result = MoveEndEffectorResult()
            self.end_effector_planner_as_.set_succeeded(result)

    def delete_action_servers(self):
        self.do_exploration_as_.__del__()
        self.move_base_as_.__del__()
        self.delete_victim_as_.__del__()
        self.gui_validate_victim_as_.__del__()
        self.data_fusion_validate_victim_as_.__del__()
        self.end_effector_planner_as_.__del__()
