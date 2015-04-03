#! /usr/bin/env python

"""
    Mocks for the tests.
"""

import rospy
import roslib
roslib.load_manifest('pandora_fsm')
from std_msgs.msg import String

from actionlib import SimpleActionServer as ActionServer
from pandora_behave import MockActionServer
from pandora_fsm import topics
from pandora_end_effector_planner.msg import MoveEndEffectorAction
from pandora_end_effector_planner.msg import MoveEndEffectorResult

TIMEOUT = 3


class MockEndEffector():
    """ A mock action server for the end effector """

    def __init__(self, *args, **kwargs):

        rospy.Subscriber('mock/cmd', String, self.receive_commands)
        self.server = ActionServer(topics.move_end_effector_planner,
                                   MoveEndEffectorAction,
                                   self.success_callback,
                                   False)
        rospy.loginfo("End effector server is starting...")
        self.server.start()
        rospy.loginfo("End effector server is waiting for goals...")

    def receive_commands(self, command):
        """  The command defines the callback that will be used from the
             ActionServer to handle incoming goals from now on.
        """
        if command.data == 'SUCCEEDED':
            self.server.execute_callback = self.success_callback
        elif command.data == 'ABORTED':
            self.server.execute_callback = self.abort_callback
        elif command.data == 'PREEMPTED':
            self.server.execute_callback = self.preempt_callback

        rospy.sleep(1)

    def preempt_callback(self, goal):

        rospy.loginfo('This goal will be preempted')
        rospy.sleep(TIMEOUT)
        self.server.set_preempted()

    def abort_callback(self, goal):

        rospy.loginfo('This goal will be aborted')
        rospy.sleep(TIMEOUT)
        self.server.set_aborted()

    def success_callback(self, goal):

        rospy.loginfo('This goal will succeed')
        rospy.sleep(TIMEOUT)
        self.server.set_succeeded()

    def __del__(self):
        self.server.__del__()

if __name__ == '__main__':
    rospy.init_node('end_effector_mock')

    server = MockEndEffector()

    rospy.spin()
