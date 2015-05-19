
""" PANDORAS's FSM Agent. """

PKG = 'pandora_fsm'

import sys
from pymitter import EventEmitter
import inspect
from functools import partial
from threading import Event
import yaml

import roslib
roslib.load_manifest(PKG)

from rospy import Subscriber, sleep
from rospy import loginfo, logerr, logwarn, logfatal
from rospkg import RosPack

from actionlib import GoalStatus

from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped

from state_manager.state_client import StateClient
from state_manager_msgs.msg import RobotModeMsg

from pandora_navigation_msgs.msg import DoExplorationGoal

from pandora_data_fusion_msgs.msg import WorldModelMsg
from pandora_data_fusion_msgs.msg import QrNotificationMsg

from pandora_rqt_gui.msg import ValidateVictimGUIResult

from pandora_end_effector_planner.msg import MoveLinearFeedback

import topics
import clients
from utils import ACTION_STATES, distance_2d
from fsm import Machine


class Agent(object):
    """ Agent implementation with a Finite State Machine.
        The agent uses a Machine instance to move through the FSM that's
        created based on a given strategy. The class describes with methods
        all the possible operations the Agent can perform.
    """

    def __init__(self, config='strategies.json', strategy='normal',
                 name='Pandora'):
        """ Initializes the agent.

        :param :name The name of the agent. Defaults to Pandora.
        :param :strategy Defines the configuration that will be loaded from
                         the Agent.
        :param :config A yaml/json file that contains the agent strategies.
                       The file should be located in the config folder of this
                       package.
        """

        # Configuration folder
        config_dir = RosPack().get_path(PKG) + '/config/'
        self.name = name
        self.strategy = strategy
        self.config = config_dir + config

        # Dispatcher for event based communication.
        self.dispatcher = EventEmitter()
        self.dispatcher.on('exploration.success', self.exploration_success)
        self.dispatcher.on('exploration.retry', self.exploration_retry)
        self.dispatcher.on('poi.found', self.poi_found)
        self.dispatcher.on('move_base.success', self.move_base_success)
        self.dispatcher.on('move_base.retry', self.move_base_retry)
        self.dispatcher.on('move_base.feedback', self.move_base_feedback)

        # SUBSCRIBERS.
        self.score_sub = Subscriber(topics.robocup_score, Int32,
                                    self.receive_score)
        self.qr_sub = Subscriber(topics.qr_notification, QrNotificationMsg,
                                 self.receive_qr_notifications)
        self.world_model_sub = Subscriber(topics.world_model, WorldModelMsg,
                                          self.receive_world_model)

        # ACTION CLIENTS.
        self.explorer = clients.Navigation(self.dispatcher)
        self.data_fusion = clients.DataFusion()
        self.control_base = clients.Control(self.dispatcher)
        self.gui_client = clients.GUI()
        self.effector = clients.Effector()
        self.linear = clients.LinearMotor()

        # State client
        loginfo('Connecting to state manager.')
        self.state_changer = StateClient()
        self.state_changer.client_initialize()
        loginfo('Connection established.')
        self.state_changer.change_state_and_wait(RobotModeMsg.MODE_OFF)

        # General information.
        self.QRs = []
        self.score = 0
        self.current_robot_pose = PoseStamped()
        self.linear_feedback = MoveLinearFeedback()
        self.exploration_mode = DoExplorationGoal.TYPE_NORMAL
        self.current_pose = PoseStamped()
        self.last_pose_goal = PoseStamped()

        # Victim information.
        self.victims_found = 0
        self.aborted_victims_ = []
        self.current_victims = []
        self.visited_victims = []
        self.deleted_victims = []
        self.target = None
        self.valid_victim_probability = 0

        # Between-transition information.
        self.is_timeout = False
        self.gui_verification = False
        self.gui_result = ValidateVictimGUIResult()

        self.state_can_change = Event()
        self.base_converged = Event()
        self.probable_victim = Event()
        self.victim_verified = Event()

        # Utility Variables
        self.IDENTIFICATION_THRESHOLD = 0.65
        self.VERIFICATION_THRESHOLD = 0.75
        self.VERIFICATION_TIMEOUT = 10
        self.STATE_CHANGE_TIMEOUT = 20
        self.MOVE_BASE_RETRIES = 0
        self.MOVE_BASE_RETRY_LIMIT = 3
        self.BASE_THRESHOLD = 0.2

        # Expose client methods to class
        setattr(self, 'test_end_effector', self.effector.test)
        setattr(self, 'park_end_effector', self.effector.park)
        setattr(self, 'preempt_end_effector', self.effector.cancel_all_goals)
        setattr(self, 'preempt_explorer', self.explorer.cancel_all_goals)
        setattr(self, 'test_linear', self.linear.test)
        setattr(self, 'scan', self.effector.scan)

        self.generate_global_state_transitions()

        self.load()

        loginfo('Agent initialized...')

    ######################################################
    #                   UTILITIES                        #
    ######################################################

    def restart_state(self):
        """ It makes the agent restart_state in the current state.
            This method should be used when something bad happened and we
            want a second chance. Possibly when a state task fails.
        """

        getattr(self, 'to_' + self.state)()

    def set_breakpoint(self, state):
        """ Stops the execution of the FSM after a given state.
            Removes the implementation from the state.

            :param :state The last state we want to go. After this state
                          the FSM will stop.
        """
        # Removing the implementation of the given state.
        self.machine.get_state(state).empty()

    def load(self):
        """ Loads the configuration file and sets up the FSM accordingly. """

        try:
            # Read the configuration file.
            with open(self.config) as file_handler:
                data = yaml.load(file_handler)
        except IOError:
            logfatal('Could not read configuration file.')
            sys.exit(1)

        try:
            states = data[self.strategy]['states']
        except KeyError:
            logfatal('%s is not a valid strategy.', self.strategy)
            sys.exit(1)

        # Setting up the FSM
        self.machine = Machine(model=self)

        # Get all the states for the given strategy.
        self.states = [state['name'] for state in states]

        # Set up states tasks.
        for state in states:
            self.machine.add_states(state['name'], on_enter=state['tasks'],
                                    on_exit=state['clean'])

        # Create the transition table.
        self.transitions = []
        for state in states:
            for transition in state['transitions']:
                self.machine.add_transition(transition['trigger'],
                                            state['name'], transition['to'],
                                            before=transition['before'],
                                            after=transition['after'],
                                            conditions=transition['conditions']
                                            )
        # Sets up the initial state
        self.machine.set_state(self.states[0])

        loginfo('FSM has been loaded.')

    def clean_up(self):
        """ Kills agent and cleans the environment. """

        self.explorer.cancel_all_goals()
        self.control_base.cancel_all_goals()
        self.linear.cancel_all_goals()
        self.effector.cancel_all_goals()
        self.gui_client.cancel_all_goals()

        loginfo('Agent is sleeping...')

    def allow_transitions(self):
        self.state_can_change.set()

    def deny_transitions(self):
        self.state_can_change.clear()

    ######################################################
    #               DISPATCHER'S CALLBACKS               #
    ######################################################

    def exploration_success(self):
        """ Called on 'exploration.success' event. The event is triggered from
            the navigation client when the current goal has succeeded.
            Enables the agent to move from the exploration state to end.
        """
        on_exploration = self.state == 'exploration'
        if self.state_can_change.is_set() and on_exploration:
            logwarn('Map covered!.')
            self.state_can_change.clear()
            self.map_covered()
        else:
            logwarn('Exploration success too soon.')

    def exploration_retry(self):
        """ Called on 'exploration.retry' event. The event is triggered from
            the navigation client when the current goal has failed and the
            agent sends again a goal.
        """
        if self.state == 'exploration':
            logwarn('Retrying exploration goal.')
            self.explore()
        else:
            logwarn('Exploration failure while not on exploration state.')

    def poi_found(self):
        """ Called on 'poi.found' event. The event is triggered when there are
            available points of interest on the received world model. Enables
            the agent to move from the exploration state to identification.
        """
        if self.state == 'exploration' and self.state_can_change.is_set():
            logwarn('A point of interest has been discovered.')
            self.state_can_change.clear()
            self.point_of_interest_found()
        else:
            logerr('Found POI outside of exploration.')

    def move_base_success(self, result):
        """ The event is triggered from the move_base client when the
            current goal has succeeded.

            :param :result The result of the action.
        """
        on_identification = self.state == 'identification'
        self.MOVE_BASE_RETRIES = 0
        if not self.state_can_change.is_set():
            logerr('Received move base success before lock release.')
            return
        elif not on_identification:
            logerr('Received move base success outside identification.')
            return

        logwarn('Approached potential victim.')
        self.state_can_change.clear()
        self.valid_victim()

    def move_base_retry(self, status):
        """ The event is triggered from the move_base client when the
            current goal has failed and the agent will send again the goal.
            After a number of failures the agent will change state and delete
            the current victim.

            :param :status The goal status.
        """
        on_identification = self.state == 'identification'
        if not self.state_can_change.is_set():
            logerr('Received move_base retry event before lock release.')
            return
        elif not on_identification:
            logerr('Received move_base retry event outside identification.')
            return

        if self.MOVE_BASE_RETRIES < self.MOVE_BASE_RETRY_LIMIT:
            # The agent tries again.
            logwarn('Retrying...%s goal', ACTION_STATES[status])
            remain = self.MOVE_BASE_RETRY_LIMIT - self.MOVE_BASE_RETRIES
            logwarn('%d remaining before aborting the current target.', remain)
            self.MOVE_BASE_RETRIES += 1
            self.control_base.move_base(self.target.victimPose)
        else:
            self.MOVE_BASE_RETRIES = 0
            # The agent changes state.
            if self.probable_victim.is_set():
                # The target is valid and the next state is hold_sensors.
                logwarn('Victim with high probability identified.')
                self.valid_victim()
            elif self.base_converged.is_set():
                logwarn('Base is close enough to the target.')
                self.valid_victim()
            else:
                # The target is not valid and we delete it.
                logwarn('Victim has been aborted.')
                self.abort_victim()

    def move_base_feedback(self, current, goal):
        """ The event is triggered from the move_base client when the
            action server sends feedback with the current pose. Given
            the goal and the current pose the agent checks if it is
            close enough to the point of interest. This will work if the
            move_base server takes too long to succeed.

            :param :current The current pose received from the action client.
            :param :goal The goal pose of the current action.
        """
        base = current.base_position
        if distance_2d(goal.pose, base.pose) < self.BASE_THRESHOLD:
            logwarn('Base converged')
            self.base_converged.set()

    ######################################################
    #               SUBSCRIBER'S CALLBACKS               #
    ######################################################

    def receive_score(self, msg):
        """ Receives the score from data fusion. """

        self.score = msg.data

    def receive_qr_notifications(self, msg):
        """ Receives QR notifications from data fusion. """

        self.QRs.append(msg)

    def receive_world_model(self, model):
        """ Receives the world model from data fusion. """

        self.current_victims = model.victims
        self.visited_victims = model.visitedVictims

        if self.current_victims:
            loginfo('@ Available POIs: ')
        for victim in self.current_victims:
            loginfo('=> #%d   (%.2f)', victim.id, victim.probability)

        if self.visited_victims:
            loginfo('@ Visited POIs: ')
        for victim in self.visited_victims:
            valid = 'valid' if victim.valid else 'not valid'
            loginfo('=> #%d   (%.2f) %s', victim.id, victim.probability, valid)

        if self.current_victims:
            if self.target:
                self.update_target_victim()
                loginfo('@ Target: #%d, (%.2f)', self.target.id,
                        self.target.probability)
                if self.target.probability > self.IDENTIFICATION_THRESHOLD:
                    self.probable_victim.set()
                if self.target.probability > self.VERIFICATION_THRESHOLD:
                    self.victim_verified.set()
            else:
                self.target = self.choose_next_victim()
                logwarn('Target acquired => #%d.', self.target.id)
            if self.state == 'exploration':
                self.dispatcher.emit('poi.found')

    def receive_linear_feedback(self, msg):
        """ Receives feedback from the linear motor. """

        self.linear_feedback = msg.feedback.linear_command_converged

    ######################################################
    #                 AGENT'S ACTIONS                    #
    ######################################################

    def reset_environment(self):
        """ Sets the environment ready for the next exploration. """

        self.gui_result.victimValid = False
        self.target = None

    def move_linear(self):
        """ Moves the linear motor the target victim. """

        self.linear.move(self.target.victimFrameId)

    def validate_victim(self):
        """ Sends information about the current target.  """

        if self.target:
            self.data_fusion.validate_victim(self.target.id,
                                             self.gui_result.victimValid)
        else:
            logerr('Reached data fusion validation without target.')

    def delete_victim(self):
        """ Send deleletion request to DataFusion about the current
            target victim.
        """
        self.data_fusion.delete_victim(self.target.id)
        self.update_victim_registry()
        self.victim_deleted()

    def update_victim_registry(self):
        if self.current_victims:
            for idx, item in enumerate(self.current_victims):
                if item.id == self.target.id:
                    break
            del self.current_victims[idx]
            self.deleted_victims.append(self.target)
            self.target = None

    def wait_identification(self):
        """ Examine if the robot can reach the target victim. """

        loginfo('Examining suspected victim...')
        self.base_client.wait_for_result()
        if self.base_client.get_state() == GoalStatus.SUCCEEDED:
            self.valid_victim()
        elif self.base_client.get_state() == GoalStatus.ABORTED:
            if self.promising_victim.is_set():
                self.promising_victim.clear()
                self.valid_victim()
            else:
                self.abort_victim()

    def wait_for_verification(self):
        """ Expect probability of the target victim to increase. """

        loginfo("Wait for the victim's probability to increase...")
        self.victim_verified.clear()
        if self.victim_verified.wait(self.VERIFICATION_TIMEOUT):
            logwarn('Victim verified.')
            self.verified()
        else:
            logwarn('Victims failed to be verified within %d secs.',
                    self.VERIFICATION_TIMEOUT)
            self.gui_result.victimValid = False
            self.timeout()

    def wait_for_operator(self):
        if self.gui_client.send_request(self.target):
            self.gui_result = self.gui_client.result()
        else:
            self.gui_result.victimValid = False

    def update_victims(self):
        """ Counts the victim if found """

        loginfo('Adding another victim...')
        if self.gui_result.victimValid:
            self.victims_found += 1

    def approach_target(self):
        """ The agent will try to approach the target's location and point
            all sensors to its direction.
        """
        loginfo('@ Approaching victim...')

        self.base_converged.clear()

        # Move base to the target.
        self.control_base.move_base(self.target.victimPose)

        # Point sensors to the target.
        self.effector.point_to(self.target.victimFrameId)

    def explore(self):
        self.explorer.explore(exploration_type=self.exploration_mode)

    def print_results(self):
        """ Prints results of the mission. """

        loginfo('The agent is shutting down...')

    ######################################################
    #                  AGENT LOGIC                       #
    ######################################################

    def update_target_victim(self):
        """ Update the current victim """

        for victim in self.current_victims:
            if victim.id == self.target.id:
                self.target = victim

    def choose_next_victim(self):
        """ Choose the next possible victim. """

        self.probable_victim.clear()
        self.victim_verified.clear()

        for victim in self.current_victims:
            if victim not in self.deleted_victims:
                return victim

    ######################################################
    #               GLOBAL STATE TRANSITIONS             #
    ######################################################

    def generate_global_state_transitions(self):
        """ Generates a function for every global state. The agent will
            be able to call this function in order to change the
            global state.

            Reads all the available modes from the RobotModeMsg and creates
            a function with the same name.
        """

        for member, value in inspect.getmembers(RobotModeMsg):
            if member.startswith('MODE_'):
                func = partial(self.global_state_transition, mode=value)
                setattr(self, member.lower(), func)

        loginfo('Global state transitions have been generated.')

    def global_state_transition(self, mode=0):
        """ Is used to generate state_transition functions.
            Given a desired mode the state_client will will try to change
            the global state.

            :param :mode A global mode from RobotModeMsg.
        """
        params = (mode, self.STATE_CHANGE_TIMEOUT)

        while True:
            success = self.state_changer.change_state_and_wait(*params)
            if success:
                break
            sleep(2)
            logerr('Failed to change the global state [%d]. Retrying...',
                   mode)
