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
# Author: Voulgarakis George

import roslib
roslib.load_manifest('pandora_fsm')
roslib.load_manifest('smach')
roslib.load_manifest('smach_ros')
import rospy
import smach_ros
import threading

from smach import StateMachine, Concurrence
from pandora_fsm.states.state_changer import MonitorModeState
from pandora_fsm.containers.data_fusion_hold import data_fusion_hold
from pandora_fsm.containers.exploration import exploration
from pandora_fsm.containers.identification import identification
from pandora_fsm.containers.robot_start import robot_start
from pandora_fsm.containers.validation import validation
from state_manager_communications.msg import robotModeMsg
from pandora_data_fusion_msgs.msg import VictimInfoMsg


def main():

    rospy.init_node('fsm')

    sm = StateMachine(outcomes=['preempted'])

    sm.userdata.target_victim = VictimInfoMsg()

    with sm:

        StateMachine.add(
            'ROBOT_START',
            robot_start(),
            transitions={
                'succeeded': 'EXPLORATION',
                'preempted': 'preempted'
            }
        )

        StateMachine.add(
            'EXPLORATION',
            exploration(),
            transitions={
                'victim_found': 'IDENTIFICATION',
                'preempted': 'preempted'
            },
            remapping={'target_victim': 'target_victim'}
        )

        StateMachine.add(
            'IDENTIFICATION',
            identification(),
            transitions={
                'victim_approached': 'DATA_FUSION_HOLD',
                'victim_aborted': 'EXPLORATION',
                'preempted': 'preempted'
            },
            remapping={'target_victim': 'target_victim'}
        )

        StateMachine.add(
            'DATA_FUSION_HOLD',
            data_fusion_hold(),
            transitions={
                'victim_verified': 'VALIDATION',
                'victim_deleted': 'EXPLORATION',
                'preempted': 'preempted'
            },
            remapping={'target_victim': 'target_victim'}
        )

        StateMachine.add(
            'VALIDATION',
            validation(),
            transitions={
                'succeeded': 'EXPLORATION',
                'preempted': 'preempted'
            },
            remapping={'target_victim': 'target_victim'}
        )

    cc = Concurrence(
        outcomes=[
            'teleoperation',
            'preempted'
        ],
        default_outcome='preempted',
        outcome_map={
            'teleoperation': {'AUTONOMOUS': 'preempted',
                              'TELEOPERATION': 'valid'},
            'preempted': {'AUTONOMOUS': 'preempted',
                          'TELEOPERATION': 'preempted'}
        },
        child_termination_cb=termination_cb
    )

    with cc:
        Concurrence.add('AUTONOMOUS', sm)
        Concurrence.add('TELEOPERATION',
                        MonitorModeState(robotModeMsg.
                                         MODE_TELEOPERATED_LOCOMOTION))

    sm_all = StateMachine(outcomes=['preempted'])

    with sm_all:

        StateMachine.add(
            'PANDORA_FSM',
            cc,
            transitions={
                'teleoperation': 'PANDORA_FSM',
                'preempted': 'preempted'
            }
        )

    sis = smach_ros.IntrospectionServer('fsm_introspection',
                                        sm_all,
                                        '/PANDORA_FSM')

    sis.start()

    smach_ros.set_preempt_handler(sm_all)

    smach_thread = threading.Thread(target=sm_all.execute)
    smach_thread.start()

    rospy.spin()
    sis.stop()


def termination_cb(outcome_map):
    return True

if __name__ == '__main__':
    main()
