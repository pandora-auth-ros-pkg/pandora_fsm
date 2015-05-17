from functools import partial
from rospy import loginfo, sleep

from actionlib import SimpleActionClient as Client

from pandora_end_effector_planner.msg import MoveEndEffectorAction
from pandora_end_effector_planner.msg import MoveEndEffectorGoal

from pandora_fsm import topics

from pandora_fsm.utils import retry_action


class Effector(object):

    """ Controls the end effector planner. """

    def __init__(self, verbose=False):
        self.verbose = verbose
        self.client = Client(topics.move_end_effector_planner,
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
        loginfo('** Waiting for the end effector action server...')
        self.client.wait_for_server()
        loginfo('** Canceling all goals on end effector.')
        self.client.cancel_all_goals()
        sleep(3)

    def point_to(self, target, center='kinect_frame'):
        """ Points end effector to a target.

        :param :target The vicitim frame ID
        :param :center Defaults to kinect_frame
        """

        goal = MoveEndEffectorGoal()
        goal.command = MoveEndEffectorGoal.TRACK
        goal.point_of_interest = target
        goal.center_point = center
        loginfo('** Waiting for end effector action server.')
        self.client.wait_for_server()
        loginfo('** Sending TRACK goal.')
        self.client.send_goal(goal)

    def scan(self):
        """ The end effector starts scanning. """

        goal = MoveEndEffectorGoal()
        goal.command = MoveEndEffectorGoal.SCAN
        loginfo('** Waiting for end effector action server.')
        self.client.wait_for_server()
        loginfo('** Sending SCAN goal.')
        self.client.send_goal(goal)
