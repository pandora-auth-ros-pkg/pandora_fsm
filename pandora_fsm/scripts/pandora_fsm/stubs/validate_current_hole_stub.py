#!/usr/bin/env python
import roslib; roslib.load_manifest('pandora_fsm')
import rospy
import actionlib
#~ from geometry_msgs import Pose
from data_fusion_communications.msg import *


class ValidateCurrentHoleActionStub:
	
	def __init__(self):
				
		self._actionStubServer = actionlib.SimpleActionServer('/data_fusion/validate_current_hole',
			ValidateCurrentHoleAction, self.executeCb, False)
		self._actionStubServer.start()
		

	def executeCb(self,goal):
		rospy.sleep(4)
		
		if self._actionStubServer.is_preempt_requested():
			self._actionStubServer.set_preempted()
		
		result = ValidateCurrentHoleResult()
		self._actionStubServer.set_succeeded(result)


if __name__ == '__main__':
	rospy.init_node('target_selector_stub')
	try:
		ValidateCurrentHoleActionStub()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass

