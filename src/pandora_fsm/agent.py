
""" PANDORAS's FSM Agent """

import time
import json

import rospy
import actionlib
import roslib
roslib.load_manifest('pandora_fsm')
from std_msgs.msg import String
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

        rospy.init_node(self.name)

        # Dummy variables
        self.victim = False
        self.is_timeout = False
        self.gui_verification = False

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

    def scan(self):
        """ Scans the area """

        print 'Scanning..'

    def preempt_scan(self):
        """ Preempts scan """

        print 'Stopping scan...'

    def move_base(self):
        """ Moves base """

        print 'Moving base...'

    def point_sensors(self):
        """ Point sensors """

        print 'Pointing sensors...'

    def wait(self):
        """ Waiting for an event """

        print 'Waiting...'

    def preempt_explore(self):
        """ Preempts exploration """

        print 'Stopping exploration...'

    def explore(self):
        """ Exploring the area. """

        print 'Exploring...'

    def stop_explorer(self):
        """ Leaving exploration mode. """

        print 'Stopping explorer...'

    def get_closer(self):
        """ The agent goes closer to the victim. """

        print 'Getting closer...'

    def move_linear(self):

        print 'Moving linear...'

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
            #self.client.cancel_goal()
            self.is_timeout = True
            self.timeout()
            print self.state

    def wake_up(self):
        """ Brings up the agent """

        print 'I am awake'

    def callback(self, data):
        """ It's called when a victim is found. """

        if data.data == 'victim':
            print 'Found one.'
            self.victim = True
            self.sub.unregister()
            self.victim_found()

    def wait_for_victim(self):
        self.sub = rospy.Subscriber('world_model', String, self.callback)

        while not self.victim:
            time.sleep(1)

    def response_from_operator(self, status, result):
        """ Receives the verification from the operator. """

        print 'Received result..'
        if self.client.get_state() == 3:
            self.gui_verification = True
            self.verification()
        else:
            self.is_timeout = True
            self.timeout()

    def wait_for_verification(self):
        """ Wait verification from the operator """

        print 'Waiting for verification'

        # Send goal to gui and wait for response.
        self.client = actionlib.SimpleActionClient('verify_victim',
                                                   VerifyVictimAction)
        goal = VerifyVictimGoal()
        self.client.wait_for_server(rospy.Duration(2))
        self.client.send_goal(goal, done_cb=self.response_from_operator)

        while not self.is_timeout and not self.gui_verification:
            time.sleep(1)
