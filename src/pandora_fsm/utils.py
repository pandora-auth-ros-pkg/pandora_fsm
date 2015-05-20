"""  Module with useful functions and tools used within the package. """

import numpy
import signal
import threading
import time

from rospy import logerr, loginfo, Duration, sleep

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
        loginfo(msg)
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
                    logerr('%s responded with %s', msg, verbose_err)
            else:
                logerr("Couldn't test %s in time...", msg)
            sleep(2)
            loginfo('Retrying...')
