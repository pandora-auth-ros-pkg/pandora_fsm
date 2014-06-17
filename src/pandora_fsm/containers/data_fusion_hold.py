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
import rospy

from smach import StateMachine, Concurrence
from pandora_fsm.states.state_changer import ChangeRobotModeState, Timer
from pandora_fsm.states.victims import VerifyVictimState, DeleteVictimState
from state_manager_communications.msg import robotModeMsg


def data_fusion_hold():

    sm = StateMachine(outcomes=['victim_verified',
                                'victim_deleted',
                                'preempted'],
                      input_keys=['target_victim'],
                      output_keys=['target_victim'])

    with sm:

        StateMachine.add(
            'ROBOT_MODE_DF_HOLD',
            ChangeRobotModeState(robotModeMsg.MODE_DF_HOLD),
            transitions={
                'succeeded': 'WAIT_FOR_DF',
                'preempted': 'preempted'
            }
        )

        cc = Concurrence(
            outcomes=[
                'victim_verified',
                'time_out',
                'preempted'
            ],
            default_outcome='preempted',
            input_keys=['target_victim'],
            output_keys=['target_victim'],
            outcome_map={
                'victim_verified': {'VERIFY_VICTIM': 'victim_verified'},
                'time_out': {'DF_TIMEOUT': 'time_out'},
                'preempted': {'VERIFY_VICTIM': 'preempted',
                              'DF_TIMEOUT': 'preempted'}
            },
            child_termination_cb=termination_cb
        )

        with cc:
            Concurrence.add('VERIFY_VICTIM', VerifyVictimState(),
                            remapping={'target_victim': 'target_victim'})
            Concurrence.add('DF_TIMEOUT', Timer(10))

        StateMachine.add(
            'WAIT_FOR_DF',
            cc,
            transitions={
                'victim_verified': 'victim_verified',
                'time_out': 'DELETE_CURRENT_VICTIM',
                'preempted': 'preempted'
            },
            remapping={'target_victim': 'target_victim'}
        )

        StateMachine.add(
            'DELETE_CURRENT_VICTIM',
            DeleteVictimState(),
            transitions={
                'succeeded': 'victim_deleted',
                'preempted': 'preempted'
            },
            remapping={'target_victim': 'target_victim'}
        )

    return sm


def termination_cb(outcome_map):
    return True
