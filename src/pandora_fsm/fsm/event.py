from collections import defaultdict


class EventData(object):
    """ Contains data to pass to a state. """

    def __init__(self, state, event, machine, model, *args, **kwargs):
        """
        :param :state The State from which the Event was triggered.
        :param :event The triggering Event.
        :param :machine The current Machine instance.
        :param :model The model/object the machine is bound to.
        :param :args, kwargs Optional positional or named arguments that will
                             be stored internally for possible later use.
        """
        self.state = state
        self.event = event
        self.machine = machine
        self.model = model
        self.args = args
        self.kwargs = kwargs

    def update(self):
        """ Updates the current State to accurately reflect the Machine. """

        self.state = self.machine.current_state


class Event(object):
    """ Events are essentially a list of callbacks attached to the model,
        enabling us to move from one state to another. When an event is
        triggered the corresponding transition is made.
    """

    def __init__(self, name, machine):
        """
        :param :name The name of the event, which is also the name of the
                     triggering callable (e.g., 'advance' implies an advance()
                     method).
        :parm :machine The current Machine instance.
        """
        self.name = name
        self.machine = machine
        self.transitions = defaultdict(list)

    def add_transition(self, transition):
        """ Adds a transition to the list of potential transitions.

        :param :transition The Transition instance to add to the list.
        """
        self.transitions[transition.source].append(transition)

    def trigger(self, *args, **kwargs):
        """ Serially execute all transitions that match the current state,
            halting as soon as one successfully completes.

        :param :args, kwargs Optional positional or named arguments that will
                             be passed onto the EventData object,
                             enabling arbitrary state information to be
                             passed on to downstream triggered functions.
        """
        state_name = self.machine.current_state.name
        if state_name not in self.transitions:
            raise MachineError(
                "Can't trigger event %s from state %s!" % (self.name,
                                                           state_name))

        # Encapsulating arguments from higher levels into an EventData object.
        event = EventData(self.machine.current_state, self, self.machine,
                          self.machine.model, *args, **kwargs)
        for transition in self.transitions[state_name]:
            if transition.execute(event):
                return True
        return False


class MachineError(Exception):
    """ General Exception for the Machine """

    def __init__(self, value):
        self.value = value

    def __str__(self):
        return repr(self.value)
