
""" PANDORAS's FSM Agent """

import json

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

        # Defines the directory with the configuration files.
        self.conf_directory = '.'

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

    def wake_up(self):
        """ Brings up the agent """

        print 'I am awake'

    def callback(self, a, b):
        """ dfd """
        print 'callback'
        self.victim_found()

    def wait_for_victim(self):
        """ Emulating victim alert """
        import signal
        import os
        import time

        signal.signal(signal.SIGALRM, self.callback)
        signal.alarm(1)
        time.sleep(2)
        print 'Alarm started'
