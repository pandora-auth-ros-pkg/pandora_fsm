"""  Module with useful functions and tools used within the package. """

import numpy
import signal
import threading
import time
import logging
from colorlog import ColoredFormatter

from rospy import Duration, sleep

FAILURE_STATES = {2: 'PREEMPTED',
                  4: 'ABORTED',
                  5: 'REJECTED',
                  8: 'RECALLED',
                  9: 'LOST'}

ACTION_STATES = {0: 'PENDING',
                 1: 'ACTIVE',
                 2: 'PREEMPTED',
                 3: 'SUCCEEDED',
                 4: 'ABORTED',
                 5: 'REJECTED',
                 6: 'PREEMPTING',
                 7: 'RECALLING',
                 8: 'RECALLED',
                 9: 'LOST'}

TERMINAL_STATES = {2: 'PREEMPTED',
                   3: 'SUCCEEDED',
                   4: 'ABORTED',
                   5: 'REJECTED',
                   8: 'RECALLED'}

GLOBAL_STATES = {0: 'OFF',
                 1: 'START_AUTONOMOUS',
                 2: 'EXPLORATION_RESCUE',
                 3: 'IDENTIFICATION',
                 4: 'SENSOR_HOLD',
                 5: 'SEMI_AUTONOMOUS',
                 6: 'TELEOPERATED_LOCOMOTION',
                 7: 'SENSOR_TEST',
                 8: 'EXPLORATION_MAPPING',
                 9: 'TERMINATING'
                 }


def listify(obj):
    """ Returns an object as a list if it isn't already. """

    return obj if isinstance(obj, (list, type(None))) else [obj]


def distance_2d(pose_a, pose_b):
    """ Calculates euclidean distance between two points in the plane.

    :param :pose_a The first Pose object.
    :param :pose_b The second Pose object.
    """

    a = numpy.array([pose_a.position.x, pose_a.position.y])
    b = numpy.array([pose_b.position.x, pose_b.position.y])

    return numpy.linalg.norm(a - b)


def distance_3d(pose_a, pose_b):
    """ Calculates euclidean distance between two points in space.

    :param :pose_a The first Pose object.
    :param :pose_b The second Pose object.
    """

    a = numpy.array([pose_a.position.x, pose_a.position.y, pose_a.position.z])
    b = numpy.array([pose_b.position.x, pose_b.position.y, pose_b.position.z])

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
        msg = '%s is interrupted by %s' % (self.task.__name__,
                                           self.after.__name__)
        raise InterruptException(msg)


def retry_action(client=None, goal=None, timeout=0, msg=''):
        """ Meta function that creates partials to test action servers.

        :param :client The instance of the client we will use for the test.
        :param :goal The goal we will send to the action server.
        :param :timeout The amount of time the client will wait before
                        attempts a retry.
        :param :msg A text for debugging.
        """
        timeout = Duration(timeout)
        log.info(msg)
        while True:
            client.wait_for_server()
            client.send_goal(goal)
            success = client.wait_for_result(timeout=timeout)
            goal_state = client.get_state()

            if success:
                if goal_state not in FAILURE_STATES.keys():
                    break
                else:
                    verbose_err = FAILURE_STATES[goal_state]
                    log.error('%s responded with %s', msg, verbose_err)
            else:
                log.error("Couldn't test %s in time...", msg)
            sleep(2)
            log.info('Retrying...')


class Timer(threading.Thread):

    """ Custom timer that executes a callback every N seconds. """

    def __init__(self, stop_flag, interval, callback, args=(), **kwargs):
        """
        :param stop_flag: Threading Event to control the timer externally.
        :param interval: An interval to execute the callback.
        :param callback: The function to execute every interval.
        :parm kwargs: Keyword parameters for the callback.

        """
        threading.Thread.__init__(self)
        self.stop_flag = stop_flag
        self.interval = interval
        self.callback = callback
        self.kwargs = kwargs
        self.args = args

    def run(self):
        """ Start the timer. """

        while not self.stop_flag.wait(self.interval):
            self.callback(*self.args, **self.kwargs)


def setup_logger():
    """ Return a logger with a default ColoredFormatter. """

    formatter = ColoredFormatter(
        "%(log_color)s%(asctime)s%(levelname)-8s %(message)s",
        datefmt='%H:%M:%S ',
        reset=True,
        log_colors={
            'DEBUG':    'blue',
            'INFO':     'green',
            'WARNING':  'bold_yellow',
            'ERROR':    'bold_red',
            'CRITICAL': 'bold_red',
        }
    )
    logger = logging.getLogger('logger')
    handler = logging.StreamHandler()
    handler.setFormatter(formatter)
    logger.addHandler(handler)
    logger.setLevel(logging.DEBUG)

    return logger

logger = setup_logger()
