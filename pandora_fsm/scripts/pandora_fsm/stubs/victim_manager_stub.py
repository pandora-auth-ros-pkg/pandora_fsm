#!/usr/bin/env python
import roslib; roslib.load_manifest('pandora_fsm')
import rospy
#~ import actionlib
#~ from geometry_msgs import Pose
from data_fusion_communications.msg import VictimFoundMsg, VictimToFsmMsg


class VictimManagerStub:
	
	def __init__(self):
				
		self.victimNotifierPub = rospy.Publisher('/data_fusion/victim_found', VictimFoundMsg)
		self.victimVerifierPub = rospy.Publisher('/data_fusion/victim_verified', VictimToFsmMsg)
		
		
		victimFoundMsg = VictimFoundMsg(victimNotificationType = VictimFoundMsg.TYPE_CAMERA )
		victimToFsmMsg = VictimToFsmMsg(probability=0.5,sensors=['1','5'])
		#~ VictimFoundMsg.victimNotificationType =  VictimFoundMsg.TYPE_THERMAL
		while not rospy.is_shutdown():
			rospy.sleep(9)
			self.victimNotifierPub.publish(victimFoundMsg)
			self.victimVerifierPub.publish(victimToFsmMsg)
				
		#~ self._actionStubServer = actionlib.SimpleActionServer('/get_victim_queue', SelectTargetAction, self.executeCb, False)
		#~ self._actionStubServer.start()
		

	#~ def executeCb(self,goal):
		#~ 
		#~ result = GetTargetResult()
		#~ self._actionStubServer.set_succeeded(result)

	


if __name__ == '__main__':
	rospy.init_node('victim_manager_stub')
	try:
		VictimManagerStub()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass

