#!/usr/bin/env python
import roslib; roslib.load_manifest('pandora_fsm')
import rospy
import actionlib
#~ from geometry_msgs import Pose
from target_selector_communications.msg import SelectTargetAction , SelectTargetResult


class SelectTargetActionStub:
	
	def __init__(self):
				
		self._actionStubServer = actionlib.SimpleActionServer('/select_target', SelectTargetAction, self.executeCb, False)
		self._actionStubServer.start()
		

	def executeCb(self,goal):
		rospy.sleep(4)
		
		if self._actionStubServer.is_preempt_requested():
			self._actionStubServer.set_preempted()
		
		result = SelectTargetResult()
		self._actionStubServer.set_succeeded(result)


if __name__ == '__main__':
	rospy.init_node('target_selector_stub')
	try:
		SelectTargetActionStub()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass

