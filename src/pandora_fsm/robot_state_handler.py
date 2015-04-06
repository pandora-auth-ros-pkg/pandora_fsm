"""
    Module containing the RobotStateHandler class.
"""

from threading import Condition

from rospy import Subscriber, Publisher
from actionlib import SimpleActionClient as Client

from state_manager_msgs.msg import RobotModeMsg, RobotModeAction

import topics


class RobotStateHandler(object):
    """ A RobotStateHandler object handles the changes in the global state.
        The object can switch between different modes
        (e.g autonomous, exploration), shutdown the robot, start the agent etc.

        The available states are:

        OFF                     = 0
        START_AUTONOMOUS        = 1
        EXPLORATION_RESCUE      = 2
        IDENTIFICATION          = 3
        SENSOR_HOLD             = 4
        SEMI_AUTONOMOUS         = 5
        TELEOPERATED_LOCOMOTION = 6
        SENSOR_TEST             = 7
        EXPLORATION_MAPPTNG     = 8
        TERMINATING             = 9

    """

    def __init__(self, agent):

        self.agent = agent

        # ActionClients
        self.state_changer = Client(topics.state_changer, RobotModeAction)

        # Lock to hold when changing the global state.
        self.current_robot_state_cond = Condition()
        self.new_robot_state_cond = Condition()
        self.current_robot_state = RobotModeMsg.MODE_OFF

    def start_agent(self):
        """ Starts the agent. """

        #TODO check for existance and throw an error
        self.agent.wake_up()

    def stop_agent(self):
        """ Stops the agent completely. Terminates the FSM. """

        self.agent.to_off()

    def reset_robot(self):
        """ Resets the robot. """
        pass

    def restart_robot(self):
        """ Restarts the robot. """
        pass


