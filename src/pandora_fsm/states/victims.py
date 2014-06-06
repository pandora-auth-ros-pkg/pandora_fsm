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
# Author: Chris Zalidis
# Author: Voulgarakis George

import roslib
roslib.load_manifest('pandora_fsm')
import rospy

from smach import State
from pandora_fsm.states.my_monitor_state import MyMonitorState
from pandora_fsm.states.my_simple_action_state import MySimpleActionState
from pandora_rqt_gui.msg import ValidateVictimGUIAction, ValidateVictimGUIGoal
from pandora_data_fusion_msgs.msg import VictimsMsg, ValidateVictimAction, \
    ValidateVictimGoal, DeleteVictimAction, DeleteVictimGoal
from pandora_fsm.robocup_agent.agent_topics import victims_topic, \
    delete_victim_topic, gui_validation_topic, data_fusion_validate_victim_topic


class NewVictimState(MyMonitorState):

    def __init__(self):
        MyMonitorState.__init__(self, victims_topic, VictimsMsg,
                                self.monitor_cb, extra_outcomes=['victim'],
                                in_keys=['target_victim'],
                                out_keys=['target_victim'])

    def monitor_cb(self, userdata, msg):
        if len(msg.victims) > 0:
            userdata[0].id = msg.victims[0].id
            userdata[0].victimPose = msg.victims[0].victimPose
            userdata[0].probability = msg.victims[0].probability
            userdata[0].sensors = msg.victims[0].sensors
            return 'victim'


class UpdateVictimState(MyMonitorState):

    def __init__(self):
        MyMonitorState.__init__(self, victims_topic, VictimsMsg,
                                self.monitor_cb,
                                extra_outcomes=['update_victim'],
                                in_keys=['target_victim'],
                                out_keys=['target_victim'])

    def monitor_cb(self, userdata, msg):
        for victim in msg.victims:
            if victim.id == userdata[0].id:
                if fabs(victim.probability - userdata[0].probability) \
                        > 0.001 or \
                    userdata[0].victimPose.pose.position.x != \
                        victim.victimPose.pose.position.x or \
                    userdata[0].victimPose.pose.position.y != \
                        victim.victimPose.pose.position.y or \
                    userdata[0].victimPose.pose.position.z != \
                        victim.victimPose.pose.position.z:
                    userdata[0].id = victim.id
                    userdata[0].victimPose = victim.victimPose
                    userdata[0].probability = victim.probability
                    userdata[0].sensors = victim.sensors
                    return 'update_victim'
                return None


class VerifyVictimState(MyMonitorState):

    def __init__(self):
        MyMonitorState.__init__(self, victims_topic, VictimsMsg,
                                self.monitor_cb,
                                extra_outcomes=['victim_verified', 'time_out'],
                                in_keys=['target_victim'],
                                out_keys=['target_victim'])

    def execute(self, userdata):
        counter = 0
        while counter < 10:
            rospy.sleep(1)

            counter = counter + 1

            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'

        return 'time_out'

    def monitor_cb(self, userdata, msg):
        for victim in msg.victims:
            if victim.id == userdata[0].id:
                if victim.probability > 0.5:
                    return 'victim_verified'
                return None


class DeleteVictimState(MySimpleActionState):

    def __init__(self):
        MySimpleActionState.__init__(self, delete_victim_topic,
                                     DeleteVictimAction,
                                     goal_cb=self.goal_cb,
                                     outcomes=['succeeded', 'preempted'],
                                     input_keys=['target_victim'],
                                     output_keys=['target_victim'])

    def goal_cb(self, userdata, goal):
        goal = DeleteVictimGoal(victimId=userdata[0].id)
        return goal


class ValidateVictimGUIState(MySimpleActionState):

    def __init__(self):
        MySimpleActionState.__init__(self, gui_validation_topic,
                                     ValidateVictimGUIAction,
                                     goal_cb=self.goal_cb,
                                     result_cb=self.result_cb,
                                     outcomes=['succeeded', 'preempted'],
                                     input_keys=['target_victim',
                                                 'validation_result'],
                                     output_keys=['target_victim',
                                                  'validation_result'])

    def goal_cb(self, userdata, goal):
        goal = ValidateVictimGUIGoal
        goal.victimFoundx = userdata[0].victimPose.pose.position.x
        goal.victimFoundy = userdata[0].victimPose.pose.position.y
        goal.probability = userdata[0].probability
        goal.sensorIDsFound = userdata[0].sensors
        return goal

    def result_cb(self, userdata, status, result):
        userdata[1] = result.victimValid
        return 'succeeded'


class ValidateVictimState(MySimpleActionState):

    def __init__(self):
        MySimpleActionState.__init__(self, data_fusion_validate_victim_topic,
                                     ValidateVictimAction,
                                     goal_cb=self.goal_cb,
                                     outcomes=['succeeded', 'preempted'],
                                     input_keys=['target_victim',
                                                'validation_result'],
                                     output_keys=['target_victim',
                                                  'validation_result'])

    def goal_cb(self, userdata, goal):
        goal = ValidateVictimGoal
        goal.victimId = userdata[0].id
        goal.victimValid = userdata[1]
        return goal
