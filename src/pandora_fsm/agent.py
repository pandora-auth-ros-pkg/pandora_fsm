
""" PANDORAS's FSM Agent. """

PKG = 'pandora_fsm'

import sys
import threading
import inspect
import yaml
from functools import partial

from pymitter import EventEmitter

import roslib
roslib.load_manifest(PKG)

from rospy import Subscriber, sleep
from rospkg import RosPack

from state_manager.state_client import StateClient
from state_manager_msgs.msg import RobotModeMsg
from pandora_exploration_msgs.msg import DoExplorationGoal
from pandora_data_fusion_msgs.msg import WorldModelMsg
from pandora_rqt_gui.msg import ValidateVictimGUIResult

import topics
import clients
import config as conf
from utils import ACTION_STATES, GLOBAL_STATES, distance_2d
from utils import logger as log
from target import Target
from fsm import Machine


class Agent(object):
    """ Agent implementation with a Finite State Machine.
        The agent uses a Machine instance to move through the FSM that's
        created based on a given strategy. The class describes with methods
        all the possible operations the Agent can perform.
    """

    def __init__(self, config='strategies.json', strategy='normal',
                 name='Pandora', testing=False, verbose=False):
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
        self.verbose = verbose
        self.testing = testing
        self.strategy = strategy
        self.config = config_dir + config

        # Dispatcher for event based communication.
        self.dispatcher = EventEmitter()

        # SUBSCRIBERS.
        self.world_model_sub = Subscriber(topics.world_model, WorldModelMsg,
                                          self.receive_world_model)

        # ACTION CLIENTS.
        self.explorer = clients.Explorer(self.dispatcher)
        self.data_fusion = clients.DataFusion()
        self.navigator = clients.Navigator(self.dispatcher)
        self.gui_client = clients.GUI()
        self.effector = clients.Effector()

        # State client
        if not self.testing:
            log.debug('Connecting to state manager.')
            self.state_changer = StateClient()
            self.state_changer.client_initialize()
            log.debug('Connection established.')
            self.state_changer.change_state_and_wait(RobotModeMsg.MODE_OFF)

        # General information.
        self.exploration_mode = DoExplorationGoal.TYPE_NORMAL

        # Victim information.
        self.available_targets = []
        self.deleted_victims = []

        self.gui_result = ValidateVictimGUIResult()

        # Between-transition information.
        self.base_converged = threading.Event()
        self.poi_found_lock = threading.Lock()

        # Utility Variables
        self.MOVE_BASE_RETRIES = 0

        self.target = Target(self.dispatcher)

        # Expose client methods to class
        setattr(self, 'test_end_effector', self.effector.test)
        setattr(self, 'park_end_effector', self.effector.park)
        setattr(self, 'preempt_end_effector', self.effector.cancel_all_goals)
        setattr(self, 'preempt_explorer', self.explorer.cancel_all_goals)
        setattr(self, 'scan', self.effector.scan)

        self.generate_global_state_transitions()

        self.load()

        log.info('Agent initialized...')

    ######################################################
    #                   UTILITIES                        #
    ######################################################

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
            log.critical('Could not read configuration file.')
            sys.exit(1)

        try:
            states = data[self.strategy]['states']
        except KeyError:
            log.critical('%s is not a valid strategy.', self.strategy)
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

        log.debug('FSM has been loaded.')

    def clean_up(self):
        """ Kills agent and cleans the environment. """

        self.explorer.cancel_all_goals()
        self.navigator.cancel_all_goals()
        self.effector.cancel_all_goals()

        log.info('Agent is sleeping...')

    def disable_events(self):
        """
        Disable all EventEmitter events. It should be called before the
        main tasks of a state.
        """
        self.dispatcher.off_all()

    ######################################################
    #               DISPATCHER'S CALLBACKS               #
    ######################################################

    def exploration_success(self):
        """ Called on 'exploration.success' event. The event is triggered from
            the exploration client when the current goal has succeeded.
            Enables the agent to move from the exploration state to end.
        """
        if self.state == 'exploration':
            log.warning('Map covered!')
            self.map_covered()
        else:
            log.warning('Exploration success caught outside of exploration.')

    def exploration_retry(self):
        """ Called on 'exploration.retry' event. The event is triggered from
            the exploration client when the current goal has failed and the
            agent sends again a goal.
        """
        if self.state == 'exploration':
            log.warning('Retrying exploration goal.')
            self.explore()
        else:
            log.warning('Exploration failure while not on exploration state.')

    def poi_found(self):
        """ Called on 'poi.found' event. The event is triggered when there are
            available points of interest on the received world model. Enables
            the agent to move from the exploration state to identification.
        """
        if self.state != 'exploration':

            # This is a bug.
            log.warning('Called poi_found outside of exploration.')
            return

        # Ensure that poi found is not called twice with the same target.
        self.poi_found_lock.acquire(False)
        log.warning('A point of interest has been discovered.')
        log.info('Pursuing target #%d.', self.target.info.id)
        self.poi_found_lock.release()

        self.point_of_interest_found()

    def move_base_success(self, result):
        """ The event is triggered from the move_base client when the
            current goal has succeeded.

            :param result: The result of the action.
        """
        self.MOVE_BASE_RETRIES = 0
        if self.state != 'identification':
            log.error('Received move base success outside identification.')
            return

        log.warning('Approached potential victim.')
        self.base_timer.cancel()
        self.valid_victim()

    def move_base_retry(self, status):
        """ The event is triggered from the move_base client when the
            current goal has failed and the agent will send again the goal.
            After a number of failures the agent will change state and delete
            the current victim.

            :param :status The goal status.
        """
        if self.state != 'identification':
            log.error('Received move_base retry event outside identification.')
            return

        if self.MOVE_BASE_RETRIES < conf.MOVE_BASE_RETRY_LIMIT:
            # The agent tries again.
            log.warning('Retrying...%s goal', ACTION_STATES[status])
            remain = conf.MOVE_BASE_RETRY_LIMIT - self.MOVE_BASE_RETRIES
            log.warning('%d remaining before aborting the current target.',
                        remain)
            self.MOVE_BASE_RETRIES += 1
            self.navigator.move_base(self.target.info.victimPose)
        else:
            self.MOVE_BASE_RETRIES = 0
            self.base_timer.cancel()

            # The agent changes state.
            if self.target.is_identified():
                # The target is valid and the next state is hold_sensors.
                log.warning('Victim with high probability identified.')
                self.valid_victim()
            elif self.base_converged.is_set():
                log.warning('Base is close enough to the target.')
                self.valid_victim()
            else:
                # The target is not valid and we delete it.
                log.warning('Victim has been aborted.')
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
        if distance_2d(goal.pose, base.pose) < conf.BASE_THRESHOLD:
            log.warning('Base converged.')
            self.base_converged.set()

    def move_base_resend(self, pose):
        """
        Send a new MoveBase goal. The old one is outdated because the
        current target's pose has been updated.

        :param pose: The new pose to go to.
        """
        if self.state == 'identification':
            log.warning('Move base goal is outdated...')
            log.debug(pose)
            self.navigator.cancel_all_goals()
            self.approach_target()

    ######################################################
    #               SUBSCRIBER'S CALLBACKS               #
    ######################################################

    def receive_world_model(self, model):
        """ Receives the world model from data fusion. """

        # If no targets are available there is nothing to do.
        if not model.victims:
            return

        if self.target.is_empty:

            # Set a new target from the available ones.
            new_target = self.choose_target(model.victims)
            self.target.set(new_target)
        else:
            # Check for invalid target acquisition.
            idx = self.target.info.id
            for target in model.victims:
                if idx == target.id:
                    break
            else:
                log.error('A non-existent target #%d has been acquired.', idx)
                self.target.clean()
                return

            # Update the current target.
            self.target.update(model.victims)
        self.dispatcher.emit('poi.found')

    ######################################################
    #                 AGENT'S ACTIONS                    #
    ######################################################

    def reset_environment(self):
        """ Sets the environment ready for the next exploration. """

        self.gui_result.victimValid = False
        self.target.clean()

    def validate_victim(self):
        """ Sends information about the current target.  """

        if not self.target.is_empty:
            self.data_fusion.validate_victim(self.target.info.id,
                                             self.gui_result.victimValid)
        else:
            log.critical('Reached data fusion validation without target.')

    def delete_victim(self):
        """ Send deleletion request to DataFusion about the current
            target victim.
        """
        self.data_fusion.delete_victim(self.target.info.id)
        self.update_victim_registry()
        self.victim_deleted()

    def update_victim_registry(self):
        if self.available_targets:
            for idx, item in enumerate(self.available_targets):
                if item.id == self.target.info.id:
                    break
            del self.available_targets[idx]
            self.deleted_victims.append(self.target.info)

    def wait_for_verification(self):
        """
        Check if the probability of the target exceeds the verification
        threshold.
        """
        log.info("Starting victim verification...")
        if self.target.verified.wait(conf.VERIFICATION_TIMEOUT):
            log.warning('Victim verified.')
            self.verified()
        else:
            log.warning('Victims failed to be verified within %d secs.',
                        conf.VERIFICATION_TIMEOUT)
            self.gui_result.victimValid = False
            self.timeout()

    def wait_for_operator(self):
        if self.gui_client.send_request(self.target.info):
            self.gui_result = self.gui_client.result()
        else:
            self.gui_result.victimValid = False

    def approach_target(self):
        """ The agent will try to approach the target's location and point
            all sensors to its direction.
        """
        log.info('Approaching victim...')

        self.base_converged.clear()

        # Move base to the target.
        self.navigator.move_base(self.target.info.victimPose)

        # Point sensors to the target.
        self.effector.point_to(self.target.info.victimFrameId)

        # Start timer to cancel all goals if the move base is unresponsive.
        self.base_timer = threading.Timer(conf.MOVE_BASE_TIMEOUT,
                                          self.timer_handler)
        self.base_timer.start()

    def explore(self):
        """
        Send exploration goal to the explorer. A different exploration
        strategy should be used depending on the state variables.
        """
        # TODO Change the exploration mode based on time,
        # how many targets are left etc.
        self.explorer.explore(exploration_type=self.exploration_mode)

    def timer_handler(self):
        if self.state == 'identification':
            log.warning('Move base is unresponsive or it takes too long.')
            self.navigator.cancel_all_goals()
            self.base_timer.cancel()
            self.abort_victim()
        else:
            log.error('Timer fired outside of identification state.')

    def print_results(self):
        """ Prints results of the mission. """

        log.info('The agent is shutting down...')

    def enable_exploration_events(self):
        """
        Enable EventEmitter events for the exploration state.
        """
        self.dispatcher.on('exploration.success', self.exploration_success)
        self.dispatcher.on('exploration.retry', self.exploration_retry)
        self.dispatcher.on('poi.found', self.poi_found)

    def enable_identification_events(self):
        """
        Enable EventEmitter events for the identfication state.
        """
        self.dispatcher.on('move_base.success', self.move_base_success)
        self.dispatcher.on('move_base.retry', self.move_base_retry)
        self.dispatcher.on('move_base.feedback', self.move_base_feedback)
        self.dispatcher.on('move_base.resend', self.move_base_resend)

    ######################################################
    #                  AGENT LOGIC                       #
    ######################################################

    def choose_target(self, targets):
        """ Choose the next possible target. """

        for target in targets:
            if target not in self.deleted_victims:
                return target

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

        log.debug('Global state transitions have been generated.')

    def global_state_transition(self, mode=0):
        """ Is used to generate state_transition functions.
            Given a desired mode the state_client will will try to change
            the global state.

            :param :mode A global mode from RobotModeMsg.
        """
        params = (mode, conf.STATE_CHANGE_TIMEOUT)

        while True:
            success = self.state_changer.change_state_and_wait(*params)
            if success:
                log.info('==> %s', GLOBAL_STATES[mode])
                break
            sleep(2)
            log.error('Failed to change the global state [%d]. Retrying...',
                      mode)
