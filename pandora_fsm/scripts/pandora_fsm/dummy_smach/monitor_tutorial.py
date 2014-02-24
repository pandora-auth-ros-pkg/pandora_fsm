#!/usr/bin/env python
import roslib; roslib.load_manifest('pandora_fsm')
import rospy
import smach
import smach_ros

from my_monitor_state import MyMonitorState

from std_msgs.msg import Empty

class bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['bar_succeeded'], input_keys=['counter'])
    def execute(self, userdata):
		rospy.sleep(3.0)
		return 'bar_succeeded'

def monitor_cb(ud, msg):
	print ud[0].counter
	return False

def main():
    rospy.init_node("monitor_example")

    sm = smach.StateMachine(outcomes=['DONE'])
    sm.userdata.counter = 1
    with sm:
        smach.StateMachine.add('FOO', MyMonitorState("/sm_reset", Empty, monitor_cb, ik=['counter']), transitions={'invalid':'BAR', 'valid':'FOO', 'preempted':'FOO'})
        smach.StateMachine.add('BAR',bar(), transitions={'bar_succeeded':'FOO'})

    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()
    sm.execute()
    rospy.spin()
    sis.stop()

if __name__=="__main__":
    main()
