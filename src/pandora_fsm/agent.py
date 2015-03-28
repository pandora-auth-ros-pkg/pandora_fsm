
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

        # Get all the states for the given strategy.
        self.states = [state['name'] for state in strategy]

        # Create the transition table.
        self.transitions = []
        for state in strategy:
            if 'transitions' in state.keys():
                for transition in state['transitions']:
                    self.transitions.append([transition['trigger'],
                                             state['name'],
                                             transition['to']])

        # Setting up the FSM
        self.machine = Machine(model=self, states=self.states,
                               transitions=self.transitions,
                               initial=self.states[0])
