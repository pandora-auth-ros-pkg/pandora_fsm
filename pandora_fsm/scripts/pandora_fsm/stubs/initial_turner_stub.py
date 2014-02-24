#!/usr/bin/env python
import roslib; roslib.load_manifest('pandora_fsm')
import rospy
import actionlib
#~ from geometry_msgs import Pose
from pandora_navigation_communications.msg import InitialTurnAction 


class InitialTurnActionStub:
	
	def __init__(self):
				
		self._actionStubServer = actionlib.SimpleActionServer('/initial_turn', InitialTurnAction, self.executeCb, False)
		self._actionStubServer.start()
		

	def executeCb(self,goal):
		rospy.sleep(10)
		
		if self._actionStubServer.is_preempt_requested():
			self._actionStubServer.set_preempted()
		self._actionStubServer.set_succeeded()


if __name__ == '__main__':
	rospy.init_node('initial_turner_stub')
	try:
		InitialTurnActionStub()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass

