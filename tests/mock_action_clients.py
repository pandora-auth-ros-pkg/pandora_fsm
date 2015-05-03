#! /usr/bin/env python

"""
    Action client mocks.
"""

from rospy import loginfo, logwarn, sleep, Subscriber, init_node, spin
import roslib
roslib.load_manifest('pandora_fsm')
from std_msgs.msg import String, Bool

from actionlib import SimpleActionServer as ActionServer
from pandora_fsm import topics

# Messages
from pandora_navigation_msgs.msg import DoExplorationAction
from pandora_end_effector_planner.msg import MoveEndEffectorAction
from pandora_end_effector_planner.msg import MoveLinearAction
from move_base_msgs.msg import MoveBaseAction
from pandora_data_fusion_msgs.msg import DeleteVictimAction
from pandora_rqt_gui.msg import ValidateVictimGUIAction
from pandora_rqt_gui.msg import ValidateVictimGUIResult
from pandora_data_fusion_msgs.msg import ValidateVictimAction


class MockActionServer(object):

    """ MockActionServer base class """

    def __init__(self, name, topic, action_type):
        """ Creating a custom mock action server."""

        self._topic = topic
        self._name = name
        self._action_type = action_type
        self.timeout = 5
        self.action_result = None

        Subscriber('mock/' + name, String, self.receive_commands)
        Subscriber('mock/gui_result', Bool, self.set_gui_result)
        self._server = ActionServer(self._topic, self._action_type,
                                    self.success, False)
        self._server.start()
        loginfo('>>> Starting ' + self._name)

    def receive_commands(self, msg):
        """ Decides the result of the next call. """

        callback, timeout = msg.data.split(':')
        self.timeout = float(timeout)
        self._server.execute_callback = getattr(self, callback)
        logwarn('>>> ' + self._name + ': Current callback -> ' + callback)
        sleep(1)

    def abort(self, goal):
        """ Aborts any incoming goal. """

        logwarn('>>> ' + self._name + ': This goal will be aborted.')
        sleep(self.timeout)
        self._server.set_aborted()

    def success(self, goal):
        """ Succeeds any incoming goal. """

        logwarn('>>> ' + self._name + ': This goal will succeed.')
        sleep(self.timeout)
        self._server.set_succeeded(self.action_result)

    def preempt(self, goal):
        """ Preempts any incoming goal. """

        logwarn('>>> ' + self._name + ': This goal will be preempted.')
        sleep(self.timeout)
        self._server.set_preempted()

    def set_gui_result(self, msg):
        """ Sets the result of the goal. """

        self.action_result = ValidateVictimGUIResult()
        logwarn('>>> The gui response will be: ' + str(msg.data))
        self.action_result.victimValid = msg.data


if __name__ == '__main__':

    init_node('mock_node')
    effector = MockActionServer('effector', topics.move_end_effector_planner,
                                MoveEndEffectorAction)
    effector = MockActionServer('linear', topics.linear_movement,
                                MoveLinearAction)
    explorer = MockActionServer('explorer', topics.do_exploration,
                                DoExplorationAction)
    move_base = MockActionServer('move_base', topics.move_base, MoveBaseAction)
    validate_victim_gui = MockActionServer('validate_gui',
                                           topics.gui_validation,
                                           ValidateVictimGUIAction)
    delete_victim = MockActionServer('delete_victim', topics.delete_victim,
                                     DeleteVictimAction)
    fusion_validate = MockActionServer('fusion_validate',
                                       topics.validate_victim,
                                       ValidateVictimAction)
    spin()
