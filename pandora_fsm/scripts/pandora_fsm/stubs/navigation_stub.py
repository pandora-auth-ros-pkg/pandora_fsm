#!/usr/bin/env python
import roslib; roslib.load_manifest('pandora_fsm')
import rospy
import actionlib
#~ from geometry_msgs import Pose
from pandora_navigation_communications.msg import * 


class MoveBaseActionStub:
	
	def __init__(self):
				
		self._actionStubServer = actionlib.SimpleActionServer('/move_base', MoveBaseAction, self.executeCb, False)
		self._actionStubServer.start()
		

	def executeCb(self,goal):
		rospy.sleep(10)
		
		if self._actionStubServer.is_preempt_requested():
			self._actionStubServer.set_preempted()
	
		self._actionStubServer.set_succeeded()


if __name__ == '__main__':
	rospy.init_node('move_base_action_stub')
	try:
		MoveBaseActionStub()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass

