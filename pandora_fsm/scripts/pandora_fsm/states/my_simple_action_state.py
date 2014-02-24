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
		
		
     
