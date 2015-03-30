
"""
    This mock node includes:

    - A node that will publish a simplified world model
      every n seconds.
    - An action server simulating a long process.
    - A service with a true or false response.
"""

import random
from datetime import datetime

import rospy
import roslib
roslib.load_manifest('pandora_fsm')
import actionlib
from std_msgs.msg import String, Int32
from pandora_fsm.msg import VerifyVictimAction, VerifyVictimGoal, \
                                                VerifyVictimResult


def world_model():
    rate = rospy.Rate(5)
    pub = rospy.Publisher('world_model', String)

    while not rospy.is_shutdown():
        data = 'victim' if datetime.now().second > 50 else 'no victim'
        pub.publish(data)
        rate.sleep()


class GUIVerificationServer(object):

    def __init__(self):
        self.result = VerifyVictimResult()
        rospy.loginfo("Server is starting...")
        self.server = actionlib.SimpleActionServer('verify_victim',
                                                   VerifyVictimAction,
                                                   execute_cb=self.execute,
                                                   auto_start=False)
        self.server.start()

    def execute(self, goal):
        rospy.loginfo('ActionServer: Received goal...')
        current_second = datetime.now().second
        if current_second > 40 and current_second < 55:
            self.server.set_succeeded(self.result)
        else:
            self.server.set_preempted()

if __name__ == '__main__':

    # Initializing the node.
    rospy.init_node('mock')

    #world_model()

    server = GUIVerificationServer()

    rospy.spin()
