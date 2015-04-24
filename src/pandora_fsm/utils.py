"""  Module with useful functions and tools used within the package. """

import numpy
import signal
import threading
import time

from rospy import logerr, loginfo

FAILURE_STATES = {2: 'PREEMPTED',
                  4: 'ABORTED',
                  5: 'REJECTED',
                  9: 'LOST'}


def listify(obj):
    """ Returns an object as a list if it isn't already. """

    return obj if isinstance(obj, (list, type(None))) else [obj]


def distance_2d(point_a, point_b):
    """ Calculates euclidean distance between two points in the plane. """

    a = (point_a.x, point_a.y)
    b = (point_b.x, point_b.y)

    return numpy.linalg.norm(a - b)


def distance_3d(point_a, point_b):
    """ Calculates euclidean distance between two points in space. """

    a = (point_a.x, point_a.y, point_a.z)
    b = (point_b.x, point_b.y, point_a.z)

    return numpy.linalg.norm(a - b)


class TimeoutException(Exception):
    pass


class InterruptException(Exception):
    pass


class TimeLimiter(object):
    """ Decorator class to limit the runtime of a function. """

    def __init__(self, timeout=10):
        """ Initializing the decorator.

        :param :timeout Time limit in seconds.
        """
        self.timeout = timeout

    def __call__(self, task):

        signal.signal(signal.SIGALRM, self.signal_handler)
        signal.alarm(self.timeout)
        self.task = task

        def wrapper(*args, **kwargs):
            try:
                self.task(*args, **kwargs)
            finally:
                signal.alarm(0)
        return wrapper

    def signal_handler(self, signum, frame):
        """ Executed if the task hasn't finished before the time limit. """

        err_msg = 'Execution of %s exceeded the time limit [%d seconds]' % (self.task.__name__,  self.timeout, )
        raise TimeoutException(err_msg)


class Interrupt(object):
    """ Decorator that stops the execution of a function and starts another.
    """

    def __init__(self, after):
        """ Initializing the decorator.

        :param :after A function to execute after the interrupt.
        """
        self.after = after

    def __call__(self, task):
        self.task = task
        signal.signal(signal.SIGINT, self.interrupt_handler)

        def wrapper(*args, **kwargs):
            try:
                self.task(*args, **kwargs)
            except InterruptException:
                self.after(*args)

        return wrapper

    def interrupt_handler(self, signum, frame):
        msg = '%s is interrupted by %s' % (self.task.__name__, self.after.__name__)
        raise InterruptException(msg)


class MultipleEvent(object):
    """ MultipleEvent object will wait for a number of different
        threading.Events. The MultipleEvent will be set if any of the events is
        set.
    """

    def __init__(self, events):
        self.events = events
        self.or_event = threading.Event()

        # Wait for all the events on separate threads.
        for name, event in self.events.items():
            threading.Thread(target=self.wait_on, args=(name, event,)).start()
            time.sleep(0.05)

        for _, event in self.events.items():
            self.wrap_event(event, self.changed)

        self.changed()

    def changed(self):
        """ Checks if any of the events is set and notifies the multi_handler
            properly.
        """
        bools = [event.is_set() for _, event in self.events.items()]
        if any(bools):
            self.or_event.set()
        else:
            self.or_event.clear()

    def wait(self):
        """ Waits for the multiple event. """

        self.or_event.wait()

    def wrap_event(self, event, changed_callback):
        """ Wraps the Event objects with new set and clear functions
            to be able to notify the multi_handler, when the state
            of an event changes.
        """
        event._set = event.set
        event._clear = event.clear
        event.changed = changed_callback
        event.set = lambda: self.wrap_set(event)
        event.clear = lambda: self.wrap_clear(event)

    def wrap_set(self, event):
        """ Event.set() wrapper to notify the multi_handler. """

        event._set()
        self.changed()

    def wrap_clear(self, event):
        """ Event.clear() wrapper to notify the multi_handler. """

        event._clear()
        self.changed()

    def wait_on(self, name, event):
        """ Debugging info for an event. """

        print "Waiting on %s..." % (name,)
        event.wait()
        print "%s fired!" % (name,)
