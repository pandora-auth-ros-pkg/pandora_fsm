from math import pi
from threading import Event
from time import sleep

from geometry_msgs.msg import PoseStamped

from actionlib import GoalStatus
from actionlib import SimpleActionClient as Client

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from tf.transformations import euler_from_quaternion, quaternion_from_euler

from pandora_fsm import topics
from pandora_fsm.utils import ACTION_STATES, TERMINAL_STATES
from pandora_fsm.utils import logger as log


class Navigator(object):

    """ Navigation client that is responsible for moving the robot
        on the map.
    """

    def __init__(self, dispatcher, verbose=False):
        self.verbose = verbose
        self.dispatcher = dispatcher
        self.client = Client(topics.move_base, MoveBaseAction)
        self.base_pending = Event()
        self.current_pose = PoseStamped()
        self.target_pose = PoseStamped()

    def cancel_all_goals(self):
        log.debug('Waiting for the Navigation action server...')
        self.client.wait_for_server()
        log.info('Canceling all goals on Navigation.')
        self.base_pending.clear()
        self.client.cancel_all_goals()
        sleep(3)

    def move_base(self, target):
        """ Move base to a point of interest.

        :param :target A PoseStamped point of interest.
        """
        roll, pitch, yaw = euler_from_quaternion([target.pose.orientation.x,
                                                  target.pose.orientation.y,
                                                  target.pose.orientation.z,
                                                  target.pose.orientation.w])
        transformend_orientation = quaternion_from_euler(roll, pitch, yaw + pi)
        target.pose.orientation.x = transformend_orientation[0]
        target.pose.orientation.y = transformend_orientation[1]
        target.pose.orientation.z = transformend_orientation[2]
        target.pose.orientation.w = transformend_orientation[3]
        target.pose.position.z = 0

        goal = MoveBaseGoal(target_pose=target)
        self.target_pose = target
        log.debug('Waiting for Navigation action server...')
        self.client.wait_for_server()
        log.info('Sending MoveBase goal.')
        log.debug('\n' + str(target.pose))
        self.base_pending.set()
        self.client.send_goal(goal, feedback_cb=self.base_feedback,
                              done_cb=self.move_base_done)

    def move_base_done(self, status, result):

        if self.base_pending.is_set():
            if status == GoalStatus.SUCCEEDED:
                log.info('Base has reached its destination.')
                self.base_pending.clear()
                self.dispatcher.emit('move_base.success', result)
            elif status in TERMINAL_STATES.keys():
                verbose_status = TERMINAL_STATES[status]
                log.error('Base has failed to move with %s.', verbose_status)
                self.base_pending.clear()
                self.dispatcher.emit('move_base.retry', status)

        if self.verbose:
            log.info('MoveBase goal status: %s', ACTION_STATES[status])

    def base_feedback(self, pose):
        self.current_pose = pose
        self.dispatcher.emit('move_base.feedback', pose, self.target_pose)
        if self.verbose:
            log.debug('Current pose updated.')
            log.debug(self.current_pose)
