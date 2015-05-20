from functools import partial
from rospy import loginfo, sleep

from actionlib import SimpleActionClient as Client

from pandora_end_effector_planner.msg import MoveLinearAction, MoveLinearGoal

from pandora_fsm import topics

from pandora_fsm.utils import retry_action


class LinearMotor(object):

    """ Controls the linear motor that moves the sensors. """

    def __init__(self, verbose=False):
        self.verbose = verbose
        self.client = Client(topics.linear_movement, MoveLinearAction)

        goal = MoveLinearGoal(command=MoveLinearGoal.TEST)
        func = partial(retry_action, client=self.client, goal=goal,
                       timeout=20, msg='Test linear motor')
        setattr(self, 'test', func)

    def cancel_all_goals(self):
        loginfo('$$ Waiting for the linear motor action server...')
        self.client.wait_for_server()
        loginfo('$$ Canceling all goals on linear motor.')
        self.client.cancel_all_goals()
        sleep(3)

    def move(self, target, center='kinect_frame'):
        """ Moves the linear motor to a specified target.

        :param :target The victimFrameId of the target.
        :param :center The center point of the target.
        """

        goal = MoveLinearGoal()
        goal.command = MoveLinearGoal.MOVE
        goal.point_of_interest = target
        goal.center_point = center
        loginfo('$$ Waiting for linear motor action server.')
        self.client.wait_for_server()
        loginfo('$$ Sending MOVE goal.')
        self.client.send_goal(goal)
