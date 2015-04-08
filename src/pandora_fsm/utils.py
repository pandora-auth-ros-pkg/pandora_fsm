

"""  Module with useful functions and tools used within the package. """

import numpy
import signal


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
