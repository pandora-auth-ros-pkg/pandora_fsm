"""
    Module containing the RobotStateHandler class.
"""

import os
import xmlrpclib
from threading import Condition

import rospy
from actionlib import SimpleActionClient as Client

from dynamic_reconfigure import find_reconfigure_services
from dynamic_reconfigure.server import Server as ParamServer
from state_manager_msgs.msg import RobotModeMsg, RobotModeAction

from pandora_fsm.cfg import FSMParamsConfig
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

    def __init__(self, name, agent, with_reconfigure=False):
        """
        :param :name The name of the handler.
        :param :agent An Agent object to handle.
        :param :with_reconfigure Option to set up a dynamic reconfigure Server.

        """
        self.agent = agent

        self.name = name

        # ActionClients
        self.state_changer = Client(topics.state_changer, RobotModeAction)

        # Communication with the master node.
        self.master = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])

        # Lock to hold when changing the global state.
        self.current_robot_state_cond = Condition()
        self.new_robot_state_cond = Condition()
        self.current_robot_state = RobotModeMsg.MODE_OFF

        # External configuration.
        if with_reconfigure:
            ParamServer(FSMParamsConfig, self.reconfigure)

    def destroy_reconfigure(self):
        """ Unregisters the set_parameters Service."""

        # Get our reconfigure service
        provider = ''.join(find_reconfigure_services())
        service = provider + '/set_parameters'

        code, msg, uri = self.master.lookupService(provider, service)
        if code:
            self.master.unregisterService(provider, service, uri)

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

    def reconfigure(self, config, level):
        """ Receive parameters from the server. """

        # TODO Maybe replace some of this with a file if nothing changes.
        self.agent.max_time = config['max_time']
        self.agent.arena_victims = config['arena_victims']
        self.agent.arena_QRs = config['arena_QRs']
        self.agent.time_passed = config['time_passed']
        self.agent.arena_type = config['arena_type']
        self.agent.initial_time = rospy.get_rostime().secs - config['time_passed']
        self.agent.valid_victim_probability = config['valid_victim_probability']
        self.agent.yellow_area = config['yellow_area']
        self.agent.yellow_black_area = config['yellow_black_area']

        # FIXME Find better names
        self.agent.aborted_victims_distance = config['aborted_victims_distance']
        self.agent.updated_victim_threshold = config['updated_victim_threshold']
        self.agent.aborted_victim_sensor_hold = config['aborted_victim_sensor_hold']

        self.agent.robot_resets = config['robot_resets']
        self.agent.robot_restarts = config['robot_restarts']

        # TODO Get the current strategy from reconfigure
        # and then load the appropriate FSM. Add option
        # in init for how to load the FSM.
        self.agent.exploration_strategy = config['exploration_strategy']
        self.goto_state = config['goto_state']

        if self.goto_state != 'no_state':
            getattr(self.agent, 'to_' + self.goto_state)()

        return config
