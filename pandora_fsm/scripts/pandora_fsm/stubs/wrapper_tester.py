#!/usr/bin/env python
import roslib; 
roslib.load_manifest('pandora_fsm')  
import rospy
import actionlib
from RoboticArm_communications.msg import *


if __name__ == '__main__':
	rospy.init_node('wrapper_tester')
	pub = rospy.Publisher("/arm/armControlTest/goal", moveArmActionGoal)
	goal = moveArmActionGoal()
	
	for i in range(5):
		goal.goal.x=i
		pub.publish(goal)
		print 'published ' + str(goal)
		rospy.sleep(2)
	

