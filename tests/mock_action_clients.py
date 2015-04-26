#! /usr/bin/env python

"""
    Action client mocks.
"""

import rospy
from rospy import loginfo, sleep, Subscriber, Timer, Duration
import roslib
roslib.load_manifest('pandora_fsm')
from std_msgs.msg import String

from actionlib import SimpleActionServer as ActionServer
from pandora_fsm import topics

# Messages
from pandora_navigation_msgs.msg import DoExplorationAction
from pandora_end_effector_planner.msg import MoveEndEffectorAction
from pandora_end_effector_planner.msg import MoveLinearAction
from move_base_msgs.msg import MoveBaseAction
from pandora_data_fusion_msgs.msg import DeleteVictimAction
from pandora_rqt_gui.msg import ValidateVictimGUIAction


class MockActionServer(object):

    """ MockActionServer base class """

    def __init__(self, name, topic, action_type):
        """ Creating a custom mock action server."""

        self._topic = topic
        self._name = name
        self._action_type = action_type
        self.timeout = 5

        rospy.Subscriber('mock/' + name, String, self.receive_commands)
        self._server = ActionServer(self._topic, self._action_type,
                                    self.success, False)
        self._server.start()
        loginfo('+ Starting ' + self._name)

    def receive_commands(self, msg):
        """ Decides the result of the next call. """

        result, timeout = msg.data.split(':')
        self.timeout = float(timeout)
        self._server.execute_callback = getattr(self, result)
        loginfo(self._name + ': Current callback -> ' + result)
        sleep(1)

    def abort(self, goal):
        """ Aborts any incoming goal. """

        loginfo(self._name + ': This goal will be aborted.')
        sleep(self.timeout)
        self._server.set_aborted()

    def success(self, goal):
        """ Succeeds any incoming goal. """

        loginfo(self._name + ': This goal will succeed.')
        sleep(self.timeout)
        self._server.set_succeeded()

    def preempt(self, goal):
        """ Preempts any incoming goal. """

        loginfo(self._name + ': This goal will be preempted.')
        sleep(self.timeout)
        self._server.set_preempted()


if __name__ == '__main__':

    rospy.init_node('mock_node')
    effector = MockActionServer('effector', topics.move_end_effector_planner,
                                MoveEndEffectorAction)
    effector = MockActionServer('linear', topics.linear_movement,
                                MoveLinearAction)
    explorer = MockActionServer('explorer', topics.do_exploration,
                                DoExplorationAction)
    move_base = MockActionServer('move', topics.move_base, MoveBaseAction)
    validate_victim_gui = MockActionServer('validate_gui',
                                           topics.gui_validation,
                                           ValidateVictimGUIAction)
    delete_victim = MockActionServer('delete_victim', topics.delete_victim,
                                     DeleteVictimAction)
    rospy.spin()
