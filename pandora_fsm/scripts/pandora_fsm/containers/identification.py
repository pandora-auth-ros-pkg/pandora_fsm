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

import roslib; roslib.load_manifest('pandora_fsm')
import rospy
import smach
import smach_ros

import pandora_fsm

from smach import State, StateMachine, Concurrence

from pandora_fsm.states.navigation import *
from pandora_fsm.states.victims import *
from pandora_fsm.states.state_changer import *

from state_manager_communications.msg import robotModeMsg
	
def cameraIdentificationSimple():
	
	def _termination_cb(outcome_map):
		return True
	
	sm = StateMachine(outcomes=['victim_approached','aborted','preempted'])
	
	with sm:
		
		StateMachine.add('GET_VICTIM',
			SelectTargetState('victim'),
			transitions={
			'succeeded':'GO_TO_VICTIM',
			'aborted':'aborted',
			'preempted':'preempted'}
		)
		
		StateMachine.add('GO_TO_VICTIM',
			MoveBaseState(),
			transitions={
			'succeeded':'victim_approached',
			'aborted':'GET_VICTIM',
			'preempted':'preempted'}
		)
		
		
	return sm	
	
