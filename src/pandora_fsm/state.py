""" Module with functions and classes related to the state object """

from utils import listify


class State(object):
    """ Implements a state of the Finite State Machine. """

    def __init__(self, name, on_enter=None, on_exit=None):
        """
        :param :name The name of the state
        :param :on_enter Optional callable(s) to trigger when a state
                         is entered. Can be either a string providing the
                         name of a callable, or a list of strings.
        :param :on_exit Optional callable(s) to trigger when a state is
                        exited. Can be either a string providing the name of a
                        callable, or a list of strings.
        """
        self.name = name
        self.on_enter = listify(on_enter) if on_enter else []
        self.on_exit = listify(on_exit) if on_exit else []

    def enter(self, event_data):
        """ Triggered when a state is entered.

        """
        for callback in self.on_enter:
            event_data.machine.callback(
                getattr(event_data.model, callback), event_data)

    def exit(self, event_data):
        """ Triggered when a state is exited.

        """
        for callback in self.on_exit:
            event_data.machine.callback(
                getattr(event_data.model, callback), event_data)

    def add_callback(self, trigger, func):
        """ Add a new enter or exit callback.

        :param :trigger The type of triggering event. Must be one of
                        'enter' or 'exit'.
        :param :func The name of the callback function.
        """
        callback_list = getattr(self, 'on_' + trigger)
        callback_list.append(func)
