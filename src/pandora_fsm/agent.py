
""" PANDORAS's FSM Agent. """

PKG = 'pandora_fsm'

import json

from math import pi

import roslib
roslib.load_manifest(PKG)
from rospy import Subscriber, Duration, Timer, sleep, loginfo, logerr
from rospkg import RosPack

from actionlib import GoalStatus
from actionlib import SimpleActionClient as Client

from std_msgs.msg import Int32, Float32

from move_base_msgs.msg import MoveBaseGoal

from tf.transformations import euler_from_quaternion, quaternion_from_euler

from pandora_navigation_msgs.msg import ArenaTypeMsg
from pandora_navigation_msgs.msg import DoExplorationAction
from pandora_navigation_msgs.msg import DoExplorationGoal

from pandora_data_fusion_msgs.msg import WorldModelMsg
from pandora_data_fusion_msgs.msg import QrNotificationMsg
from pandora_data_fusion_msgs.msg import ValidateVictimAction
from pandora_data_fusion_msgs.msg import ValidateVictimGoal
from pandora_data_fusion_msgs.msg import DeleteVictimAction
from pandora_data_fusion_msgs.msg import DeleteVictimGoal

from move_base_msgs.msg import MoveBaseAction

from pandora_rqt_gui.msg import ValidateVictimGUIAction
from pandora_rqt_gui.msg import ValidateVictimGUIGoal

from pandora_end_effector_planner.msg import MoveEndEffectorAction
from pandora_end_effector_planner.msg import MoveEndEffectorGoal
from pandora_end_effector_planner.msg import MoveLinearActionFeedback

import topics
from machine import Machine


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
        """

        # Configuration folder
        config_dir = RosPack().get_path(PKG) + '/config/'
        self.name = name

        self.strategy = strategy

        self.config = config_dir + config

        # Subscribers.
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

        # Action clients.
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

        self.victim = False
        self.valid_victims = 0
        self.new_victims = []
        self.target_victim = None

        # TODO should load from a config file
        self.valid_victim_probability = 0
        self.exploration_mode = 0
        self.linear_feedback = False

        self.is_timeout = False
        self.gui_verification = False
        self.result = False

        self.load()

    def load(self):
        """ Loads the configuration file and sets up the FSM accordingly. """

        try:
            # Read the configuration file.
            with open(self.config) as file_handler:
                data = json.load(file_handler)
        except IOError, err:
            raise err

        states = data[self.strategy]['states']

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
            if 'transitions' in state.keys():
                for transition in state['transitions']:
                    self.machine.add_transition(transition['trigger'],
                                                state['name'],
                                                transition['to'])

                    self.machine.set_state(self.states[0])


    ######################################################
    #####           SUBSCRIBER'S CALLBACKS           #####
    ######################################################


    def receive_score(self, score):
        """ Receives the score from data fusion. """
        pass

    def receive_qr_notifications(self, data):
        """ Receives QR notifications from data fusion. """
        pass

    def receive_world_model(self, data):
        """ Receives the world model as published by data fusion. """
        pass

    def receive_linear_feedback(self, data):
        """ Receives feedback from the linear motor. """
        pass

    def receive_area_covered(self, data):
        """ Receives notifications about the covered area. """
        pass

    ######################################################
    #####              AGENT'S ACTIONS               #####
    ######################################################

    def boot(self):
        """ Boots up the system. """

        loginfo('System boot!')

        # Test end effector planner
        self.test_end_effector_planner()
        if self.end_effector_client.get_state() == GoalStatus.ABORTED:
            logerr('Failed to test end effector.')
            self.to_boot()

        # Park end effector planner
        self.park_end_effector_planner()
        if self.end_effector_client.get_state() == GoalStatus.ABORTED:
            logerr('Failed to park end effector.')
            self.to_boot()

        self.booted()

    def test_end_effector_planner(self):
        """ Tests end effector with action client. """

        loginfo('Testing end effector...')
        goal = MoveEndEffectorGoal(command=MoveEndEffectorGoal.TEST)
        self.end_effector_client.send_goal_and_wait(goal)
        self.end_effector_client.wait_for_result()

    def park_end_effector_planner(self):
        """ Parks end effector with action client. """

        loginfo('Trying to park end effector...')
        goal = MoveEndEffectorGoal(command=MoveEndEffectorGoal.PARK)
        self.end_effector_client.send_goal_and_wait(goal)
        self.end_effector_client.wait_for_result()

    def scan(self):
        """ Scans the area. """

        loginfo('Start scanning...')

        self.preempt_end_effector_planner()
        goal = MoveEndEffectorGoal(command=MoveEndEffectorGoal.SCAN)
        self.end_effector_client.send_goal(goal)

    def preempt_scan(self):
        """ Preempts scan. """

        loginfo('Stop scanning...')
        self.end_effector_client.cancel_all_goals()

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
        self.base_client.send_goal(goal, feedback_cb=self.base_feedback)

    ######################################################
    #####              ACTION'S CALLBACKS            #####
    ######################################################

    def base_feedback(self, feedback):
        """ Receives action feedback from the move base server. """
        pass

    def point_sensors(self):
        """ Point sensors. """

        loginfo('Pointing sensors...')
        goal = MoveEndEffectorGoal()
        goal.command = MoveEndEffectorGoal.TRACK
        goal.point_of_interest = self.target_victim.victimFrameId
        # TODO Move 'kinect_frame' to a variable
        goal.center_point = 'kinect_frame'
        self.end_effector_client.send_goal(goal)

    def wait_identification(self):
        """ Examines if there is a victim. """

        loginfo('Examining suspected victim...')

        if self.base_client.get_state() == GoalStatus.SUCCEEDED:
            self.valid_victim()
        elif self.base_client.get_state() == GoalStatus.ABORTED:
            self.abort_victim()

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

    def wait_for_verification(self):
        """ Waiting for victim to be verified. """

        for victim in self.new_victims:
            if victim.id == self.target_victim.id:
                face = False
        for sensor in victim.sensors:
            if sensor == 'FACE':
                face = True
        if victim.probability > self.valid_victim_probability and \
           len(victim.sensors) >= 2 or face:
            self.target_victim = victim
            self.verified()
        else:
            goal = ValidateVictimGoal()
            goal.victimId = self.target_victim.id
            goal.victimValid = False
            result_ = self.fusion_validate_client.send_goal_and_wait(goal, 60)
        if result_ == 'DONE':
            self.verified()
        else:
            self.timeout()
        sleep(1.)

    def receive_arena_type(self, data):
        """ Receives the type of the arena. """
        pass

    def preempt_explore(self):
        """ Preempts exploration. """

        loginfo('Stopping exploration...')
        self.explorer.cancel_all_goals()

    def explore(self):
        """ Exploring the area. """

        loginfo('Exploring...')
        goal = DoExplorationGoal(exploration_type=self.exploration_mode)
        self.explorer.send_goal(goal, feedback_cb=self.explorer_feedback,
                                done_cb=self.explorer_done)

    def explorer_feedback(self, feedback):
        """ Position feedback. """
        self.current_robot_pose = feedback.base_position

    def explorer_done(self, status, result):
        """ Exploration finished. """
        self.map_covered()

    def victim_callback(self, data):
        """ It's called when a victim is found. """
        pass

    def stop_explorer(self):
        """ Leaving exploration mode. """

        loginfo('Stopping explorer...')
        self.exploration_mode = -1

    def start_timer(self, period):

        loginfo('Starting timer...')

        duration = Duration(secs=period)
        Timer(duration, self.timer_handler, oneshot=True)

    def timer_handler(self, event):
        """ Timer expired. """

        loginfo('Timer expired...')

        if self.state == 'closeup':
            self.is_timeout = True
            self.timeout()
            loginfo(self.state)

    def abort_end_effector(self):
        """ Stops the end effector and starts exploring again. """

        loginfo('Aborting end effector...')
        self.preempt_end_effector_planner()
        self.park_end_effector_planner()
        if self.end_effector_client.get_state() == GoalStatus.ABORTED:
            logerr("Failed to park end effector")
            self.to_boot()

    def preempt_end_effector_planner(self):
        """ Preemts the last goal on the end effector planer. """
        pass

    def delete_victim(self):
        """ Deletes the victim that failed to be identified. """

        loginfo('Deleting victim...')
        goal = DeleteVictimGoal(victimId=self.target_victim.id)
        self.delete_victim_client.send_goal(goal)
        self.delete_victim_client.wait_for_result()
        if self.delete_victim_client.get_state() == GoalStatus.ABORTED:
            loginfo("Failed to delete victim")
            self.to_boot()
        self.victim_deleted()

    def victim_classification(self):
        """ Classifies the victim last attempted to identify """

        loginfo('Classifying victim...')

        goal = ValidateVictimGoal()
        goal.victimId = self.target_victim.id
        goal.victimValid = self.result.victimValid
        self.fusion_validate_client.send_goal(goal)
        self.fusion_validate_client.wait_for_result()
        if self.fusion_validate_client.get_state() == GoalStatus.ABORTED:
            loginfo("failed to delete victim")
            self.to_boot()
        self.victim_classified()

    def operator_confirmation(self):
        """ Waits for operator to confirm the victim found """

        loginfo('DID WE FIND IT?????')
        goal = ValidateVictimGUIGoal()
        goal.victimFoundx = self.target_victim.victimPose.pose.position.x
        goal.victimFoundy = self.target_victim.victimPose.pose.position.y
        goal.probability = self.target_victim.probability
        goal.sensorIDsFound = self.target_victim.sensors
        self.gui_validate_client.send_goal(goal)
        self.gui_validate_client.wait_for_result()
        self.result = self.gui_validate_client.get_result()
        self.responded()

    def update_victims(self):
        """ Counts the victim if found """

        loginfo('AND ONE')
        self.valid_victims += 1

    def response_from_operator(self, result):
        """ Receives the verification from the operator. """

        loginfo('Received result..')
