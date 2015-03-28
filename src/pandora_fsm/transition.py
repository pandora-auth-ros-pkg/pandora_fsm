""" Module about the transition """

from utils import listify


class Transition(object):
    """ Implements a transition on the Finite State Machine """

    class Condition(object):

        def __init__(self, func, target=True):
            self.func = func
            self.target = target

        def check(self, model):
            """ Check whether the condition passes.

            :param :model The data model attached to the current Machine.
            """
            return getattr(model, self.func)() == self.target

    def __init__(self, source, dest, conditions=None, unless=None, before=None,
                 after=None):
        """
        :param :source The name of the source State.
        :param :dest The name of the destination State.
        :param :conditions Condition(s) that must pass in order for the
                           transition to take place. Either a string
                           providing the name of a callable, or a list of
                           callables. For the transition to occur, ALL
                           callables must return True.
        :param :unless Condition(s) that must return False in order for the
                       transition to occur. Behaves just like conditions arg
                       otherwise.
        :param :before Callbacks to trigger before the transition.
        :param :after Callbacks to trigger after the transition.
        """
        self.source = source
        self.dest = dest
        self.before = [] if before is None else listify(before)
        self.after = [] if after is None else listify(after)

        self.conditions = []
        if conditions is not None:
            for condition in listify(conditions):
                self.conditions.append(self.Condition(condition))
        if unless is not None:
            for unless_condition in listify(unless):
                self.conditions.append(self.Condition(unless_condition,
                                                      target=False))

    def execute(self, event_data):
        """ Execute the transition.

        :param :event An instance of class EventData.
        """
        machine = event_data.machine
        for condition in self.conditions:
            if not condition.check(event_data.model):
                return False

        for func in self.before:
            machine.callback(getattr(event_data.model, func), event_data)
        machine.get_state(self.source).exit(event_data)
        machine.set_state(self.dest)
        event_data.update()
        machine.get_state(self.dest).enter(event_data)
        for func in self.after:
            machine.callback(getattr(event_data.model, func), event_data)
        return True

    def add_callback(self, trigger, func):
        """ Add a new before or after callback.

        :param :trigger The type of triggering event. Must be one of
                        'before' or 'after'.
        :param :func The name of the callback function.
        """
        callback_list = getattr(self, trigger)
        callback_list.append(func)


