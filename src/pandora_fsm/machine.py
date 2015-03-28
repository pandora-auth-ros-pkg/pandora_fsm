""" Module with the Machine class """


from functools import partial
from collections import OrderedDict

from utils import listify
from state import State
from event import Event
from transition import Transition


class Machine(object):
    """ The Machine is high level abstraction that holds together the states,
        the transitions, the events and the model.
    """

    def __init__(self, model=None, states=None, initial=None, transitions=None,
                 send_event=False, auto_transitions=True,
                 ordered_transitions=False):
        """
        :param :model The object whose states we want to manage. If None,
                      the current Machine instance will be used the model
                      (i.e., all triggering events will be attached to the
                      Machine itself).
        :param :states A list of valid states. Each element can be either a
                       string or a State instance. If string, a new generic
                       State instance will be created that has the same name
                       as the string.
        :param :initial The initial state of the Machine. Defaults to 'initial'
        :param :transitions An optional list of transitions. Each element is a
                            dictionary of named arguments to be passed onto the
                            Transition initializer.
        :param :send_event When True, any arguments passed to trigger methods
                           will be wrapped in an EventData object, allowing
                           indirect and encapsulated access to data.
                           When False, all positional and keyword arguments
                           will be passed directly to all callback methods.
        :param :auto_transitions When True (default), every state will
                                 automatically have an associated to_{state}()
                                 convenience trigger in the base model.
        :param :ordered_transitions Convenience argument that calls
                                    add_ordered_transitions() at the end of
                                    initialization if set to True.
        """
        self.model = self if model is None else model
        self.states = OrderedDict()
        self.events = {}
        self.current_state = None
        self.send_event = send_event
        self.auto_transitions = auto_transitions

        if initial is None:
            self.add_states('initial')
            initial = 'initial'
        self._initial = initial

        if states is not None:
            self.add_states(states)

        self.set_state(self._initial)

        if transitions is not None:
            transitions = listify(transitions)
            for transition in transitions:
                if isinstance(transition, list):
                    self.add_transition(*transition)
                else:
                    self.add_transition(**transition)

        if ordered_transitions:
            self.add_ordered_transitions()

    def is_state(self, state):
        """ Check whether the current state matches the named state. """
        return self.current_state.name == state

    def get_state(self, state):
        """ Return the State instance with the passed name. """
        if state not in self.states:
            raise ValueError("State '%s' is not a registered state." % state)
        return self.states[state]

    def set_state(self, state):
        """ Set the current state.

        :param :state A State instance or the name of a state.
        """
        if isinstance(state, basestring):
            state = self.get_state(state)
        self.current_state = state
        self.model.state = self.current_state.name

    def add_state(self, *args, **kwargs):
        """ Alias for add_states. """
        self.add_states(*args, **kwargs)

    def add_states(self, states, on_enter=None, on_exit=None):
        """ Add new state(s).

        :param :state A list, a State instance, the name of a new state,
                      or a dict with keywords to pass on to the State
                      initializer. If a list, each element can be of any of the
                      latter three types.
        :param :on_enter Callbacks to trigger when the state is entered.
                         Only valid if first argument is string.
        :param :on_exit Callbacks to trigger when the state is exited.
                        Only valid if first argument is string.
        """

        states = listify(states)
        for state in states:
            if isinstance(state, basestring):
                state = State(state, on_enter=on_enter, on_exit=on_exit)
            elif isinstance(state, dict):
                state = State(**state)
            self.states[state.name] = state
            setattr(self.model, 'is_%s' %
                    state.name, partial(self.is_state, state.name))
            state_name = state.name
            if self != self.model and hasattr(
                    self.model, 'on_enter_' + state_name):
                state.add_callback('enter', 'on_enter_' + state_name)
            if self != self.model and hasattr(
                    self.model, 'on_exit_' + state_name):
                state.add_callback('exit', 'on_exit_' + state_name)
        # Add automatic transitions after all states have been created
        if self.auto_transitions:
            for state in self.states.keys():
                self.add_transition('to_%s' % state, '*', state)

    def add_transition(self, trigger, source, dest, conditions=None,
                       unless=None, before=None, after=None):
        """ Create a new Transition instance and add it to the internal list.

        :param :trigger The name of the method that will trigger the
                        transition. This will be attached to the currently
                        specified model (e.g., passing trigger='advance' will
                        create a new advance() method in the model that triggers
                        the transition.)
        :param :source The name of the source state--i.e., the state we are
                       transitioning away from.
        :param :dest The name of the destination State--i.e., the state
                     we are transitioning into.
        :param :conditions Condition(s) that must pass in order for the
                           transition to take place. Either a list providing the
                           name of a callable, or a list of callables.
                           For the transition to occur, ALL callables must
                           return True.
        :param :unless Condition(s) that must return False in order for the
                       transition to occur. Behaves just like conditions arg
                       otherwise.
        :param :before Callables to call before the transition.
        :param :after Callables to call after the transition.

        """
        if trigger not in self.events:
            self.events[trigger] = Event(trigger, self)
            setattr(self.model, trigger, self.events[trigger].trigger)

        if isinstance(source, basestring):
            source = list(self.states.keys()) if source == '*' else [source]

        for src in source:
            transition = Transition(src, dest, conditions, unless, before,
                                    after)
            self.events[trigger].add_transition(transition)

    def add_ordered_transitions(self, states=None, trigger='next_state',
                                loop=True, loop_includes_initial=True):
        """ Add a set of transitions that move linearly from state to state.
        Args:
            states (list): A list of state names defining the order of the
                transitions. E.g., ['A', 'B', 'C'] will generate transitions
                for A --> B, B --> C, and C --> A (if loop is True). If states
                is None, all states in the current instance will be used.
            trigger (string): The name of the trigger method that advances to
                the next state in the sequence.
            loop (boolean): Whether or not to add a transition from the last
                state to the first state.
            loop_includes_initial (boolean): If no initial state was defined in
                the machine, setting this to True will cause the _initial state
                placeholder to be included in the added transitions.
        """
        if states is None:
            states = list(self.states.keys())  # need to listify for Python3
        if len(states) < 2:
            raise MachineError("Can't create ordered transitions on a Machine "
                               "with fewer than 2 states.")
        for i in range(1, len(states)):
            self.add_transition(trigger, states[i - 1], states[i])
        if loop:
            if not loop_includes_initial:
                states.remove(self._initial)
            self.add_transition(trigger, states[-1], states[0])

    def callback(self, func, event_data):
        """ Trigger a callback function, possibly wrapping it in an EventData
        instance.
        Args:
            func (callable): The callback function.
            event_data (EventData): An EventData instance to pass to the
                callback (if event sending is enabled) or to extract arguments
                from (if event sending is disabled).
        """
        if self.send_event:
            func(event_data)
        else:
            func(*event_data.args, **event_data.kwargs)

    def __getattr__(self, name):
        terms = name.split('_')
        if terms[0] in ['before', 'after']:
            name = '_'.join(terms[1:])
            if name not in self.events:
                raise MachineError('Event "%s" is not registered.' % name)
            return partial(self.events[name].add_callback, terms[0])

        elif name.startswith('on_enter') or name.startswith('on_exit'):
            state = self.get_state('_'.join(terms[2:]))
            return partial(state.add_callback, terms[1])



class MachineError(Exception):
    """ General Exception for the Machine """

    def __init__(self, value):
        self.value = value

    def __str__(self):
        return repr(self.value)
