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
from pandora_fsm.states.state_changer import ChangeRobotModeState
from pandora_fsm.states.navigation import DoExplorationState
from pandora_fsm.states.victims import NewVictimState
from state_manager_msgs.msg import RobotModeMsg


def exploration():

    sm = StateMachine(outcomes=['victim_found', 'preempted'],
                      input_keys=['target_victim'],
                      output_keys=['target_victim'])

    with sm:

        StateMachine.add(
            'ROBOT_MODE_EXPLORATION',
            ChangeRobotModeState(RobotModeMsg.MODE_EXPLORATION),
            transitions={
                'succeeded': 'EXPLORATION_WITH_VICTIMS',
                'preempted': 'preempted'
            }
        )

        cc = Concurrence(
            outcomes=[
                'exploration_aborted',
                'victim_found',
                'preempted'
            ],
            default_outcome='preempted',
            input_keys=['target_victim'],
            output_keys=['target_victim'],
            outcome_map={
                'exploration_aborted': {'EXPLORE': 'aborted'},
                'victim_found': {'NEW_VICTIM_MONITOR': 'victim'},
                'preempted': {'EXPLORE': 'preempted',
                              'NEW_VICTIM_MONITOR': 'preempted'}
            },
            child_termination_cb=termination_cb
        )

        with cc:
            Concurrence.add('EXPLORE', DoExplorationState())
            Concurrence.add('NEW_VICTIM_MONITOR', NewVictimState(),
                            remapping={'target_victim': 'target_victim'})

        StateMachine.add(
            'EXPLORATION_WITH_VICTIMS',
            cc,
            transitions={
                'exploration_aborted': 'EXPLORATION_WITH_VICTIMS',
                'victim_found': 'victim_found',
                'preempted': 'preempted'
            },
            remapping={'target_victim': 'target_victim'}
        )

    return sm


def termination_cb(outcome_map):
    return True
