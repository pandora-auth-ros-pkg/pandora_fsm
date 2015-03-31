
""" PANDORAS's FSM Agent """

import time
import json

import rospy
import roslib
roslib.load_manifest('pandora_fsm')

from actionlib import SimpleActionClient as Client

from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Float32

from pandora_navigation_msgs.msg import ArenaTypeMsg
from pandora_navigation_msgs.msg import DoExplorationAction

from pandora_data_fusion_msgs.msg import WorldModelMsg
from pandora_data_fusion_msgs.msg import VictimInfoMsg
from pandora_data_fusion_msgs.msg import QrNotificationMsg
from pandora_data_fusion_msgs.msg import ValidateVictimAction
from pandora_data_fusion_msgs.msg import DeleteVictimAction

from move_base_msgs.msg import MoveBaseAction

from pandora_rqt_gui.msg import ValidateVictimGUIAction
from pandora_rqt_gui.msg import ValidateVictimGUIGoal

from pandora_end_effector_planner.msg import MoveEndEffectorAction
from pandora_end_effector_planner.msg import MoveEndEffectorGoal
from pandora_end_effector_planner.msg import MoveLinearFeedback
from pandora_end_effector_planner.msg import MoveLinearActionFeedback

import topics
from pandora_fsm.msg import VerifyVictimAction, VerifyVictimGoal
from machine import Machine


class Agent(object):
    """ Agent implementation with a Finite State Machine. """

    def __init__(self, strategy='normal', name='Pandora',
                 config='strategies.json'):
        """ Initializes the agent.

        :param :name The name of the agent. Defaults to Pandora.
        :param :strategy Defines the configuration that will be loaded from
                         the Agent.
        """
        self.name = name

        self.strategy = strategy

        self.config = config

        # Subscribers.
        rospy.Subscriber(topics.arena_type, ArenaTypeMsg,
                         self.arena_type_callback)
        rospy.Subscriber(topics.robocup_score, Int32, self.score_callback)
        rospy.Subscriber(topics.qr_notification, QrNotificationMsg,
                         self.qr_notification_callback)
        rospy.Subscriber(topics.area_covered, Float32,
                         self.area_covered_callback)
        rospy.Subscriber(topics.world_model, WorldModelMsg,
                         self.world_model_callback)
        rospy.Subscriber(topics.linear_movement_action_feedback,
                         MoveLinearActionFeedback,
                         self.linear_feedback_callback)

        # Action clients.
        self.explorer_client = Client(topics.do_exploration,
                                      DoExplorationAction)
        self.base_client = Client(topics.move_base, MoveBaseAction)
        self.delete_victim_client = Client(topics.delete_victim,
                                           DeleteVictimAction)
        self.gui_validate_client = Client(topics.gui_validation,
                                          ValidateVictimGUIAction)
        self.fusion_validate_client = Client(topics.validate_victim,
                                             ValidateVictimAction)
        self.end_effector_client = Client(topics.move_end_effector_planner,
                                          MoveEndEffectorAction)

        # Dummy variables
        self.victim = False
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

        strategy = data[self.strategy]['states']

        # Setting up the FSM
        self.machine = Machine(model=self)

        # Get all the states for the given strategy.
        self.states = [state['name'] for state in strategy]

        # Set up states tasks.
        for state in strategy:
            self.machine.add_states(state['name'], on_enter=state['tasks'],
                                    on_exit=state['clean'])

        # Create the transition table.
        self.transitions = []
        for state in strategy:
            if 'transitions' in state.keys():
                for transition in state['transitions']:
                    self.machine.add_transition(transition['trigger'],
                                                state['name'],
                                                transition['to'])

        self.machine.set_state(self.states[0])

    def boot(self):
        """ Boots up the system. """

        print 'System boot!'
        #test end effector planner
        goal = MoveEndEffectorGoal(command=MoveEndEffectorGoal.TEST)
		self.end_effector_planner_ac_.send_goal_and_wait(goal)
		self.end_effector_planner_ac_.wait_for_result()
		if self.end_effector_planner_ac_.get_state() == GoalStatus.ABORTED:
			self.to_boot()
			rospy.loginfo("failed to test end effector")
		#park end effector planner
		self.park_end_effector_planner()
		if self.end_effector_planner_ac_.get_state() == GoalStatus.ABORTED:
			rospy.loginfo("failed to park end effector")
			self.to_boot()
		else 
			self.booted()
	
	#made as a separate function because it is used in 2 different state
	#functions, boot and abort
	def park_end_effector_planner(self)
		goal = MoveEndEffectorGoal(command=MoveEndEffectorGoal.PARK)
		self.end_effector_planner_ac_.send_goal_and_wait(goal)
		self.end_effector_planner_ac_.wait_for_result()
	
    def scan(self):
        """ Scans the area """

        print 'Scanning..'
        self.preempt_end_effector_planner()
		goal = MoveEndEffectorGoal(command=MoveEndEffectorGoal.SCAN)
		self.end_effector_planner_ac_.send_goal(goal)
		#previous implementation didn't wait for a result
		

    def preempt_scan(self):
        """ Preempts scan """

        print 'Stopping scan...'
        self.end_effector_planner_ac_.cancel_all_goals()

    def move_base(self):
        """ Moves base """

        print 'Moving base...'
        victim = self.target_victim_.victimPose
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
        self.move_base_ac_.send_goal(goal, feedback_cb=self.feedback_cb)
        

    def point_sensors(self):
        """ Point sensors """

        print 'Pointing sensors...'
        goal = MoveEndEffectorGoal()
		goal.command = MoveEndEffectorGoal.TRACK
		goal.point_of_interest = self.target_victim_.victimFrameId
		goal.center_point = "kinect_frame"
		self.end_effector_planner_ac_.send_goal(goal)
	
	def wait_identification(self):
		""" Examines if there is a victim """
		
		print 'Examining suspected victim..'
		
		if self.move_base_ac_.get_state() == GoalStatus.SUCCEEDED:
			self.valid_victim()
		elif self.move_base_ac_.get_state() == GoalStatus.ABORTED:
			self.abort_victim()
	
	def move_linear(self)
		""" Moving linear to identify the victim """
		
		print "Moving linear..."
		self.preempt_end_effector_planner()
		goal = MoveEndEffectorGoal()
		goal.command = MoveEndEffectorGoal.LAX_TRACK
		goal.point_of_interest = self.target_victim_.victimFrameId
		goal.center_point = "kinect_frame"
		self.agent_.linear_feedback_ = False
		self.agent_.end_effector_planner_ac_.send_goal(goal)
	
	def wait_for_verification(self)
		""" Waiting for victim to be verified """ 
		for victim in self.agent_.new_victims_:
			if victim.id == self.target_victim_.id:
				face = False
				for sensor in victim.sensors:
					if sensor == 'FACE':
						face = True
			if victim.probability > self.agent_.valid_victim_probability_ and \
									(len(victim.sensors) >= 2 or face):
				self.agent_.target_victim_ = victim
				self.verified()
				else:
					goal = ValidateVictimGoal()
					goal.victimId = self.agent_.target_victim_.id
					goal.victimValid = False
					result_ = self.agent_.data_fusion_validate_victim_ac_.\
											send_goal_and_wait(goal, 60)
				if result_ == 'DONE':
					self.verified()
				else:
					self.timeout()
				rospy.sleep(1.)

    
    def preempt_explore(self):
        """ Preempts exploration """

        print 'Stopping exploration...'
        self.do_exploration_ac_.cancel_all_goals()
        

    def explore(self):
        """ Exploring the area. """

        print 'Exploring...'
        goal = DoExplorationGoal(exploration_type = self.current_exploration_mode)
		self.do_exploration_ac_.send_goal(goal, feedback_cb=self.feedback_cb,
									      done_cb=self.done_cb)
	
	
	def feedback_cb(self, feedback):
		""" Position feedback """
        self.current_robot_pose_ = feedback.base_position

    def done_cb(self, status, result):
		""" Exploration finished """
        self.map_covered()								  
	
	def wait_for_victim(self):
        self.sub = rospy.Subscriber('world_model', String, self.victim_callback)

        while not self.victim:
            time.sleep(1)
	
	def victim_callback(self, data):
        """ It's called when a victim is found. """

        if data.data == 'victim':
            print 'Found one.'
            self.victim = True
            self.sub.unregister()
            self.victim_found()

    def stop_explorer(self):
        """ Leaving exploration mode. """

        print 'Stopping explorer...'
        self.current_exploration_mode_ = -1
   

	def start_timer(self):

        print 'Starting timer...'

        duration = rospy.Duration(5)
        rospy.Timer(duration, self.timer_handler, oneshot=True)

    def timer_handler(self, event):
        """ Timer expired """

        print 'timer expired...'

        # If the timer expires on the closeup state
        # trigger the timeout event.
        if self.state == 'closeup':
            self.is_timeout = True
            self.timeout()
            print self.state

    def abort_end_effector(self):
		""" Stops the end effector and starts exploring again """
		
		print 'Aborting end effector...'
		self.preempt_end_effector_planner()
		self.park_end_effector_planner()
		if self.end_effector_planner_ac_.get_state() == GoalStatus.ABORTED:
			rospy.loginfo("failed to park end effector")
			self.to_boot()
	
	def delete_victim(self):
		""" Deletes the victim that failed to be identified """
		
		print 'Deleting victim...'
		goal = DeleteVictimGoal(victimId=self.target_victim_.id)
		self.delete_victim_ac_.send_goal(goal)
		self.delete_victim_ac_.wait_for_result()
		if self.delete_victim_ac_.get_state() == GoalStatus.ABORTED:
			rospy.loginfo("failed to delete victim")
			self.to_boot()
		self.victim_deleted()
	
	def victim_classification(self):
		""" Classifies the victim last attempted to identify """
		
		print 'Classifying victim...'
		goal = ValidateVictimGoal()
		goal.victimId = self.agent_.target_victim_.id
		goal.victimValid = result.victimValid
		self.data_fusion_validate_victim_ac_.send_goal(goal)
		self.data_fusion_validate_victim_ac_.wait_for_result()
		if self.data_fusion_validate_victim_ac_.get_state() == GoalStatus.ABORTED:
			rospy.loginfo("failed to delete victim")
			self.to_boot()
		self.victim_classified()
	
	def operator_confirmation(self):
		""" Waits for operator to confirm the victim found """
		
		print 'DID WE FIND IT?????'
		goal = ValidateVictimGUIGoal()
		goal.victimFoundx = self.target_victim_.victimPose.pose.position.x
		goal.victimFoundy = self.target_victim_.victimPose.pose.position.y
		goal.probability = self.target_victim_.probability
		goal.sensorIDsFound = self.target_victim_.sensors
		self.gui_validate_victim_ac_.send_goal(goal)
		self.gui_validate_victim_ac_.wait_for_result()
		self.result = self.agent_.gui_validate_victim_ac_.get_result()
		self.responded()
	
	def update_victims(self):
		""" Counts the victim if found """
		
		print 'AND ONE'
		self.valid_victims_ += 1

	
	
    def response_from_operator(self, status, result):
        """ Receives the verification from the operator. """

        print 'Received result..'
        if self.client.get_state() == 3:
            self.gui_verification = True
            self.verification()
        else:
            self.is_timeout = True
            self.timeout()

    """def wait_for_verification(self):
        

        print 'Waiting for verification'

        # Send goal to gui and wait for response.
        self.client = Client('verify_victim', ValidateVictimGUIAction)
        goal = ValidateVictimGUIGoal()
        self.client.wait_for_server(rospy.Duration(2))
        self.client.send_goal(goal, done_cb=self.response_from_operator)

        while not self.is_timeout and not self.gui_verification:
            time.sleep(1)
	"""
