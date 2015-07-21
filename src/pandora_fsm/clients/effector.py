from functools import partial
from time import sleep

from actionlib import SimpleActionClient as Client

from pandora_end_effector_controller.msg import MoveEndEffectorAction
from pandora_end_effector_controller.msg import MoveEndEffectorGoal

from pandora_fsm import topics

from pandora_fsm.utils import retry_action
from pandora_fsm.utils import logger as log


class Effector(object):

    """ Controls the end effector planner. """

    def __init__(self, verbose=False):
        self.verbose = verbose
        self.client = Client(topics.move_end_effector_controller,
                             MoveEndEffectorAction)

        goal = MoveEndEffectorGoal(command=MoveEndEffectorGoal.TEST)
        func = partial(retry_action, client=self.client, goal=goal,
                       timeout=20, msg='Test end effector ')
        setattr(self, 'test', func)
        goal = MoveEndEffectorGoal(command=MoveEndEffectorGoal.PARK)
        func = partial(retry_action, client=self.client, goal=goal,
                       timeout=20, msg='Park end effector ')
        setattr(self, 'park', func)

    def cancel_all_goals(self):
        log.debug('Waiting for the EndEffector action server...')
        self.client.wait_for_server()
        log.info('Canceling all goals on EndEffector.')
        self.client.cancel_all_goals()
        sleep(3)

    def point_to(self, target, center='/picam_frame'):
        """ Points end effector to a target.

        :param target: The victim frame ID.
        :param center: The center of the frame the we will use.
        """

        goal = MoveEndEffectorGoal()
        goal.command = MoveEndEffectorGoal.TRACK
        goal.point_of_interest = target
        goal.center_point = center
        log.debug('Waiting for the EndEffector action server.')
        self.client.wait_for_server()
        log.info('Sending TRACK goal.')
        self.client.send_goal(goal)

    def slowly_point_to(self, target, center='/picam_frame'):
        """
        Points end effector to a target, slow enough so the image
        is staying still.

        :param target: The victim frame ID.
        :param center: The center of the frame the we will use.
        """

        goal = MoveEndEffectorGoal()
        goal.command = MoveEndEffectorGoal.LAX_TRACK
        goal.point_of_interest = target
        goal.center_point = center
        log.debug('Waiting for the EndEffector action server.')
        self.client.wait_for_server()
        log.info('Sending LAX TRACK goal.')
        self.client.send_goal(goal)

        sleep(5)

    def scan(self):
        """ The end effector starts scanning. """

        goal = MoveEndEffectorGoal()
        goal.command = MoveEndEffectorGoal.SCAN
        log.debug('Waiting for the EndEffector action server.')
        self.client.wait_for_server()
        log.info('Sending SCAN goal.')
        self.client.send_goal(goal)
