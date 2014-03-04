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

__all__ = ['MyMonitorState']

class MyMonitorState(smach.State):
    """A state that will check a given ROS topic with a condition function. 
    """
    def __init__(self, topic, msg_type, cond_cb, extra_outcomes=[], in_keys=[], out_keys=[], max_checks=-1):
        smach.State.__init__(self,outcomes=['invalid','preempted']+extra_outcomes, input_keys=in_keys, output_keys=out_keys)

        self._topic = topic
        self._msg_type = msg_type
        self._cond_cb = cond_cb
        self._max_checks = max_checks
        self._n_checks = 0
        
        self.return_outcome = None

        self._trigger_cond = threading.Condition()

    def execute(self, ud):
        self._n_checks = 0

        self._sub = rospy.Subscriber(self._topic, self._msg_type, self._cb, callback_args=[ud])

        self._trigger_cond.acquire()
        self._trigger_cond.wait()
        self._trigger_cond.release()

        self._sub.unregister()

        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        if self._max_checks > 0 and self._n_checks >= self._max_checks:
            return 'invalid'

        return self.return_outcome

    def _cb(self, msg, ud):
        self._n_checks += 1
        try:
			
			self.return_outcome = self._cond_cb(ud, msg)
			
			if (self._max_checks > 0 and self._n_checks >= self._max_checks) or (self.return_outcome is not None):
				self._trigger_cond.acquire()
				self._trigger_cond.notify()
				self._trigger_cond.release()
        except:
            rospy.logerr("Error thrown while executing condition callback %s" % str(self._cond_cb))
            self._trigger_cond.acquire()
            self._trigger_cond.notify()
            self._trigger_cond.release()

    def request_preempt(self):
        smach.State.request_preempt(self)
        self._trigger_cond.acquire()
        self._trigger_cond.notify()
        self._trigger_cond.release()
