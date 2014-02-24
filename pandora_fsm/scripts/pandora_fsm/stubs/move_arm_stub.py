#!/usr/bin/env python
import roslib; roslib.load_manifest('pandora_fsm')
import rospy
import actionlib
#~ from geometry_msgs import Pose
from robotic_arm_communications.msg import MoveArmAction, MoveArmResult


class MoveArmActionStub:
	
	def __init__(self):
				
		self._actionStubServer = actionlib.SimpleActionServer('/arm/arm_control', MoveArmAction, self.executeCb, False)
		self._actionStubServer.start()
		

	def executeCb(self,goal):
		
		if goal.motionType == goal.TYPE_FOLLOW_POINT:
			
			while not rospy.is_shutdown():
				
				if self._actionStubServer.is_preempt_requested():
					self._actionStubServer.set_preempted()
		else:
			rospy.sleep(4)
			if self._actionStubServer.is_preempt_requested():
				self._actionStubServer.set_preempted()
			result = MoveArmResult()
			self._actionStubServer.set_succeeded(result)


if __name__ == '__main__':
	rospy.init_node('move_arm_stub')
	try:
		MoveArmActionStub()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass

