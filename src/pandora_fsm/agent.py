
""" PANDORAS's FSM Agent. """

PKG = 'pandora_fsm'

import sys
import os
import inspect
from functools import partial
from threading import Event
import signal
import yaml
from math import pi

import roslib
roslib.load_manifest(PKG)

from rospy import Subscriber, Duration, sleep
from rospy import loginfo, logerr, logwarn, logfatal
from rospkg import RosPack

from actionlib import GoalStatus
from actionlib import SimpleActionClient as Client

from std_msgs.msg import Int32, Float32, String
from geometry_msgs.msg import PoseStamped

from state_manager.state_client import StateClient
from state_manager_msgs.msg import RobotModeMsg

from tf.transformations import euler_from_quaternion, quaternion_from_euler

from pandora_navigation_msgs.msg import ArenaTypeMsg
from pandora_navigation_msgs.msg import DoExplorationAction, DoExplorationGoal

from pandora_data_fusion_msgs.msg import WorldModelMsg
from pandora_data_fusion_msgs.msg import QrNotificationMsg
from pandora_data_fusion_msgs.msg import ValidateVictimAction
from pandora_data_fusion_msgs.msg import ValidateVictimGoal
from pandora_data_fusion_msgs.msg import DeleteVictimAction, DeleteVictimGoal

from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction

from pandora_rqt_gui.msg import ValidateVictimGUIAction, ValidateVictimGUIGoal
from pandora_rqt_gui.msg import ValidateVictimGUIResult

from pandora_end_effector_planner.msg import MoveEndEffectorAction
from pandora_end_effector_planner.msg import MoveEndEffectorGoal
from pandora_end_effector_planner.msg import MoveLinearActionFeedback
from pandora_end_effector_planner.msg import MoveLinearFeedback, MoveLinearGoal
from pandora_end_effector_planner.msg import MoveLinearAction

import topics
from machine import Machine
from utils import FAILURE_STATES, MultipleEvent


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

        # SUBSCRIBERS.
        self.arena_sub = Subscriber(topics.arena_type, ArenaTypeMsg,
                                    self.receive_arena_type)
        self.score_sub = Subscriber(topics.robocup_score, Int32,
                                    self.receive_score)
        self.qr_sub = Subscriber(topics.qr_notification, QrNotificationMsg,
                                 self.receive_qr_notifications)
        self.area_coverage_sub = Subscriber(topics.area_covered, Float32,
                                            self.receive_area_covered)
        self.world_model_sub = Subscriber(topics.world_model, WorldModelMsg,
                                          self.receive_world_model)
        self.linear_sub = Subscriber(topics.linear_movement_action_feedback,
                                     MoveLinearActionFeedback,
                                     self.receive_linear_feedback)
        self.interrupts = Subscriber(topics.agent_interrupt, String,
                                     self.receive_interrupts)

        # ACTION CLIENTS.
        self.explorer = Client(topics.do_exploration, DoExplorationAction)
        self.base_client = Client(topics.move_base, MoveBaseAction)
        self.delete_victim_client = Client(topics.delete_victim,
                                           DeleteVictimAction)
        self.gui_validate_client = Client(topics.gui_validation,
                                          ValidateVictimGUIAction)
        self.fusion_validate_client = Client(topics.validate_victim,
                                             ValidateVictimAction)
        self.end_effector_client = Client(topics.move_end_effector_planner,
                                          MoveEndEffectorAction)
        self.linear_client = Client(topics.linear_movement, MoveLinearAction)

        # State client
        loginfo('Connecting to state manager.')
        self.state_changer = StateClient()
        self.state_changer.client_initialize()
        loginfo('Connection established.')
        self.state_changer.change_state_and_wait(RobotModeMsg.MODE_OFF)

        # Attributes to help in the decision making.

        # Arena information.
        self.current_arena = ArenaTypeMsg.TYPE_YELLOW
        self.yellow_arena_area_explored = False
        self.yellow_black_arena_area_explored = False

        # General information.
        self.QRs = []
        self.score = 0
        self.current_robot_pose = PoseStamped()
        self.linear_feedback = MoveLinearFeedback()
        self.exploration_mode = DoExplorationGoal.TYPE_NORMAL

        # Victim information.
        self.victims_found = 0
        self.aborted_victims_ = []
        self.current_victims = []
        self.visited_victims = []
        self.target_victim = None
        self.valid_victim_probability = 0

        # Between-transition information.
        self.is_timeout = False
        self.gui_verification = False
        self.gui_result = ValidateVictimGUIResult()

        # Communication between threads (callbacks and the main thread).

        # Is set when an unidentified victim is present in the world model.
        self.potential_victim = Event()

        # Is set when the robot cant reach the target victim, but it has
        # high probability.
        self.promising_victim = Event()

        # Is set when the operator validates the target victim.
        self.recognized_victim = Event()
        self.explored = Event()

        # Utility Variables
        self.IDENTIFICATION_THRESHOLD = 0.65
        self.VERIFICATION_THRESHOLD = 0.75
        self.VERIFICATION_TIMEOUT = 10
        self.STATE_CHANGE_TIMEOUT = 20
        self.END_EFFECTOR_TIMEOUT = Duration(10)

        self.generate_global_state_transitions()

        # Generate synchronous transactions to test the action servers.
        goal = MoveEndEffectorGoal(command=MoveEndEffectorGoal.TEST)
        func = partial(self.test_action_server,
                       client=self.end_effector_client,
                       goal=goal,
                       timeout=self.END_EFFECTOR_TIMEOUT,
                       msg='End effector test')
        setattr(self, 'test_end_effector', func)

        goal = MoveEndEffectorGoal(command=MoveEndEffectorGoal.PARK)
        func = partial(self.test_action_server,
                       client=self.end_effector_client,
                       goal=goal,
                       timeout=self.END_EFFECTOR_TIMEOUT,
                       msg='Park end effector')
        setattr(self, 'park_end_effector', func)

        goal = MoveLinearGoal(command=MoveLinearGoal.TEST)
        func = partial(self.test_action_server,
                       client=self.linear_client,
                       goal=goal,
                       timeout=self.END_EFFECTOR_TIMEOUT,
                       msg='Linear test')
        setattr(self, 'test_linear', func)
        loginfo('Synchronous transactions have been generated.')

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
        except IOError, err:
            raise err

        try:
            states = data[self.strategy]['states']
        except KeyError, err:
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
        self.base_client.cancel_all_goals()
        self.linear_client.cancel_all_goals()
        self.end_effector_client.cancel_all_goals()
        self.fusion_validate_client.cancel_all_goals()
        self.gui_validate_client.cancel_all_goals()
        self.delete_victim_client.cancel_all_goals()

        self.to_off()

        loginfo('Agent is sleeping...')

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

        loginfo('World model updated...')
        self.current_victims = model.victims
        self.visited_victims = model.visitedVictims

        if self.current_victims:
            self.potential_victim.set()
            if self.target_victim:
                self.update_target_victim()

                if self.target_victim.probability > self.IDENTIFICATION_THRESHOLD:
                    self.promising_victim.set()
                else:
                    self.promising_victim.clear()
                if self.target_victim.probability > self.VERIFICATION_THRESHOLD:
                    self.recognized_victim.set()
                else:
                    self.recognized_victim.clear()
            else:
                self.target_victim = self.choose_next_victim()
        else:
            self.potential_victim.clear()


    def receive_linear_feedback(self, msg):
        """ Receives feedback from the linear motor. """

        self.linear_feedback = msg.feedback.linear_command_converged

    def receive_area_covered(self, msg):
        """ Receives notifications about the covered area. """
        pass

    def receive_arena_type(self, msg):
        """ Receives the type of the arena. """
        pass

    def receive_interrupts(self, msg):
        """ Receives interrupt commands. Stops the current task and
            executes another.
        """
        os.kill(os.getpid(), signal.SIGINT)

    ######################################################
    #                 AGENT'S ACTIONS                    #
    ######################################################

    def reset_environment(self):
        """ Sets the environment ready for the next exploration. """

        self.gui_result.victimValid = False
        self.target_victim = None

    def test_action_server(self, client=None, goal=None, timeout=0, msg=''):
        """ Meta function that creates partials to test action servers.

        :param :client The instance of the client we will use for the test.
        :param :goal The goal we will send to the action server.
        :param :timeout The amount of time the client will wait before
                        attempts a retry.
        :param :msg A text for debugging.
        """

        loginfo(msg)
        while True:
            client.wait_for_server()
            client.send_goal(goal)
            success = client.wait_for_result(timeout=timeout)
            goal_state = client.get_state()

            if success:
                if goal_state not in FAILURE_STATES.keys():
                    break
                else:
                    verbose_err = FAILURE_STATES[goal_state]
                    logerr('%s responded with %s', msg, verbose_err)
            else:
                logerr("Couldn't test %s in time...", msg)
            sleep(2)
            loginfo('Retrying...')

    def point_sensors(self):
        """ Points end effector to the target. """

        self.preempt_end_effector_planner()
        goal = MoveEndEffectorGoal()
        goal.command = MoveEndEffectorGoal.TRACK
        goal.point_of_interest = self.target_victim.victimFrameId
        goal.center_point = 'kinect_frame'
        self.end_effector_client.wait_for_server()
        self.end_effector_client.send_goal(goal)

    def scan(self):
        """ Scans the area. """

        loginfo('Start scanning...')

        self.preempt_end_effector_planner()
        goal = MoveEndEffectorGoal(command=MoveEndEffectorGoal.SCAN)
        self.end_effector_client.wait_for_server()
        self.end_effector_client.send_goal(goal)

    def move_linear(self):
        """ Moving linear to identify the victim. """

        loginfo('Moving linear...')

        self.preempt_end_effector_planner()
        goal = MoveEndEffectorGoal()
        goal.command = MoveEndEffectorGoal.LAX_TRACK
        goal.point_of_interest = self.target_victim.victimFrameId
        goal.center_point = "kinect_frame"
        self.linear_feedback = False
        self.end_effector_client.send_goal(goal)

        loginfo('Send new goal to linear...')

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

    def wait_for_victim(self):
        """ The agent stops until a candidate victim is found."""

        loginfo('Waiting for victim...')

        # Setting up the events to wait for.
        events_to_wait = {'Victim found': self.potential_victim,
                          'Exploration done': self.explored}

        MultipleEvent(events_to_wait).wait()

        if self.potential_victim.is_set():
            loginfo('Victim found...')

            # Reseting the flag.
            # FIXME
            self.potential_victim.clear()

            # Setting the target victim.
            self.target_victim = self.choose_next_victim()

            # Changing state.
            self.victim_found()
        elif self.explored.is_set():
            loginfo('The explorer has finished...')

            # Changing state.
            self.map_covered()
        else:
            logerr('> wait_for_victim: None of the events is set but the agent \
                    passed the wait. Possible error with the MultipleEvent.')

            # Entering again on the current state.
            getattr(self, 'to_' + self.state)()

    def wait_for_verification(self):
        """ Expect probability of the target victim to increase. """

        loginfo("Wait for the victim's probability to increase...")
        self.recognized_victim.clear()
        if self.recognized_victim.wait(self.VERIFICATION_TIMEOUT):
            self.verified()
        else:
            self.gui_result.victimValid = False
            self.timeout()

    def move_base(self):
        """ Moves base to a point of intereset. """

        # FIXME Improve the structure of the function
        loginfo('Moving base...')
        victim = self.target_victim.victimPose
        roll, pitch, yaw = euler_from_quaternion([victim.pose.orientation.x,
                                                  victim.pose.orientation.y,
                                                  victim.pose.orientation.z,
                                                  victim.pose.orientation.w])

        transformed_orientation = quaternion_from_euler(roll, pitch, yaw + pi)
        victim.pose.orientation.x = transformed_orientation[0]
        victim.pose.orientation.y = transformed_orientation[1]
        victim.pose.orientation.z = transformed_orientation[2]
        victim.pose.orientation.w = transformed_orientation[3]

        victim.pose.position.z = 0

        goal = MoveBaseGoal(target_pose=victim)
        self.base_client.wait_for_server()
        self.base_client.send_goal(goal, feedback_cb=self.base_feedback)

    def explore(self):
        """ Exploring the area. """

        loginfo('Exploring...')
        self.explored.clear()
        goal = DoExplorationGoal(exploration_type=self.exploration_mode)
        self.explorer.wait_for_server()
        self.explorer.send_goal(goal, feedback_cb=self.explorer_feedback,
                                done_cb=self.explorer_done)

    def delete_victim(self):
        """ Deletes the victim that failed to be identified. """

        goal = DeleteVictimGoal(victimId=self.target_victim.id)

        # FIXME Abort after 5 retries
        while True:
            logwarn('Deleting victim ' + str(goal.victimId))

            self.delete_victim_client.wait_for_server()
            self.delete_victim_client.send_goal(goal)
            self.delete_victim_client.wait_for_result()
            goal_state = self.delete_victim_client.get_state()

            if goal_state is GoalStatus.SUCCEEDED:
                break
            logerr('Failed to delete victim')
            loginfo('Retrying...')
        loginfo('Victim deletion succeeded.')

        # Changing state.
        self.victim_deleted()

    def wait_for_operator(self):
        """ Waits for operator to confirm the victim. """

        loginfo('Waiting for confirmation...')
        goal = ValidateVictimGUIGoal()
        goal.victimFoundx = self.target_victim.victimPose.pose.position.x
        goal.victimFoundy = self.target_victim.victimPose.pose.position.y
        goal.probability = self.target_victim.probability
        goal.sensorIDsFound = self.target_victim.sensors

        self.gui_validate_client.wait_for_server()
        self.gui_validate_client.send_goal(goal)
        self.gui_validate_client.wait_for_result()
        self.gui_result = self.gui_validate_client.get_result()

        # Changing state.
        self.operator_responded()

    def update_victims(self):
        """ Counts the victim if found """

        loginfo('Adding another victim...')
        if self.gui_result.victimValid:
            self.victims_found += 1

    def victim_classification(self):
        """ Send info about the target victim to data fusion, to
            updated its registry. If the victim has been validated
            from the operator, the target will be marked as visited/valid.
            On the other hand the victim will be marked as visited/invalid.
        """

        loginfo('Sending victim info to data fusion...')

        goal = ValidateVictimGoal()
        goal.victimId = self.target_victim.id
        goal.victimValid = self.gui_result.victimValid
        self.fusion_validate_client.wait_for_server()
        self.fusion_validate_client.send_goal(goal)
        self.fusion_validate_client.wait_for_result()
        if self.fusion_validate_client.get_state() == GoalStatus.ABORTED:
            loginfo('Failed to delete victim.')
            self.restart_state()
        else:
            self.victim_classified()

    def response_from_operator(self, result):
        """ Receives the verification from the operator. """

        loginfo('Received result..')

    def print_results(self):
        """ Prints results of the mission. """

        loginfo('The agent is shutting down...')

    ######################################################
    #              ACTION'S CALLBACKS                    #
    ######################################################

    def explorer_feedback(self, feedback):
        """ Position feedback. """
        self.current_robot_pose = feedback.base_position

    def explorer_done(self, status, result):
        """ Results from the exploration. """

        if status == GoalStatus.SUCCEEDED:
            loginfo('Exploration has finished successfully...')
            self.explored.set()
        elif status == GoalStatus.ABORTED:
            loginfo('Exploration has been aborted...')
            self.explored.clear()
        elif status == GoalStatus.REJECTED:
            loginfo('Exploration has been rejected...')
            self.explored.clear()

    def victim_callback(self, data):
        """ It's called when a victim is found. """
        pass

    def base_feedback(self, feedback):
        """ Receives action feedback from the move base server. """
        pass

    ######################################################
    #                  AGENT LOGIC                       #
    ######################################################

    def update_target_victim(self):
        """ Update the current victim """

        for victim in self.current_victims:
            if victim.id == self.target_victim.id:
                self.target_victim = victim

    def choose_next_victim(self):
        """ Choose the next possible victim """

        return self.current_victims[0]

    ######################################################
    #               PREEMPTS AND ABORTS                  #
    ######################################################

    def preempt_exploration(self):
        """ Preempts exploration. """

        loginfo('Stopping exploration...')
        self.exploration_mode = DoExplorationGoal.TYPE_NORMAL
        self.explorer.wait_for_server()
        self.explorer.cancel_all_goals()
        self.explorer.wait_for_result()
        self.explored.clear()
        loginfo('Exploration stopped...')

    def preempt_move_base(self):
        """ Stops any move base goals. """

        loginfo('Stopping base...')
        self.base_client.wait_for_server()
        self.base_client.cancel_all_goals()
        self.base_client.wait_for_result()
        loginfo('Base stopped...')

    def abort_end_effector(self):
        """ Stops the end effector and starts exploring again. """

        loginfo('Aborting end effector...')
        self.preempt_end_effector_planner()
        self.park_end_effector_planner()
        if self.end_effector_client.get_state() == GoalStatus.ABORTED:
            logerr("Failed to park end effector")
            self.restart_state()
        loginfo('End effector aborted...')

    def preempt_end_effector_planner(self):
        """ Preempts the last goal on the end effector planer. """

        loginfo('Preempting end effector...')
        # self.end_effector_client.wait_for_server()
        self.end_effector_client.cancel_all_goals()
        self.end_effector_client.wait_for_result()
        loginfo('End effector preempted...')

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
