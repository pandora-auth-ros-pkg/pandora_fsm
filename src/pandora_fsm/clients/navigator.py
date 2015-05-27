from math import pi
from threading import Event
import rospy
from rospy import logerr, loginfo, sleep

from geometry_msgs.msg import PoseStamped

from actionlib import GoalStatus
from actionlib import SimpleActionClient as Client

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from tf.transformations import euler_from_quaternion, quaternion_from_euler

from pandora_fsm import topics
from pandora_fsm.utils import ACTION_STATES, TERMINAL_STATES


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
        loginfo('++ Waiting for the move base action server...')
        self.client.wait_for_server()
        loginfo('++ Canceling all goals on move base.')
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
        loginfo('++ Waiting for move base action server...')
        self.client.wait_for_server()
        loginfo('++ Sending move base goal.')
        loginfo(target.pose)
        self.base_pending.set()
        self.client.send_goal(goal, feedback_cb=self.base_feedback,
                              done_cb=self.move_base_done)

    def move_base_done(self, status, result):

        if self.base_pending.is_set():
            if status == GoalStatus.SUCCEEDED:
                loginfo('++ Base has reached its destination.')
                self.base_pending.clear()
                self.dispatcher.emit('move_base.success', result)
            elif status in TERMINAL_STATES.keys():
                verbose_status = TERMINAL_STATES[status]
                logerr('++ Base has failed to move with %s.', verbose_status)
                self.base_pending.clear()
                self.dispatcher.emit('move_base.retry', status)

        if self.verbose:
            loginfo('++ Move base goal status: %s', ACTION_STATES[status])

    def base_feedback(self, pose):
        self.current_pose = pose
        self.dispatcher.emit('move_base.feedback', pose, self.target_pose)
        if self.verbose:
            loginfo('++ Current pose updated.')
            loginfo(self.current_pose)
