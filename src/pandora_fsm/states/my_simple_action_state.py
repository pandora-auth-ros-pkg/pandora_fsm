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

import roslib; roslib.load_manifest('smach_ros')
import rospy

import threading
import traceback

import smach
import smach_ros

__all__ = ['MySimpleActionState']

class MySimpleActionState(smach_ros.SimpleActionState):

    def __init__(self, # Action info
	            action_name,
	            action_spec, 
	            # Default goal 
	            goal = None,
	            goal_key = None,
	            goal_slots = [],
	            goal_cb = None,
	            goal_cb_args = [],
	            goal_cb_kwargs = {},
	            # Result modes
	            result_key = None,
	            result_slots = [],
	            result_cb = None,
	            result_cb_args = [],
	            result_cb_kwargs = {},
	            # Keys
	            input_keys = [],
	            output_keys = [],
	            outcomes = [],
	            # Timeouts
	            exec_timeout = None,
	            preempt_timeout = rospy.Duration(60.0),
	            server_wait_timeout = rospy.Duration(60.0)):
		
		smach_ros.SimpleActionState.__init__(self,
											# Action info
								            action_name,
								            action_spec, 
								            # Default goal 
								            goal,
								            goal_key,
								            goal_slots,
								            goal_cb,
								            goal_cb_args,
								            goal_cb_kwargs,
								            # Result modes
								            result_key,
								            result_slots,
								            result_cb,
								            result_cb_args,
								            result_cb_kwargs,
								            # Keys
								            input_keys,
								            output_keys,
								            outcomes,
								            # Timeouts
								            exec_timeout,
								            preempt_timeout,
								            server_wait_timeout)
								            
		self._outcomes = outcomes
		
		
     
