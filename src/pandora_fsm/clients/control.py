from math import pi
from threading import Event
from rospy import logerr, loginfo, sleep

from geometry_msgs.msg import PoseStamped

from actionlib import GoalStatus
from actionlib import SimpleActionClient as Client

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from tf.transformations import euler_from_quaternion, quaternion_from_euler

from pandora_fsm import topics
from pandora_fsm.utils import ACTION_STATES


class Control(object):

    """ Control client to move the robot on the map. """

    def __init__(self, verbose=False):
        self.verbose = verbose
        self.base_client = Client(topics.move_base, MoveBaseAction)
        self.base_succeded = Event()
        self.base_pending = Event()
        self.current_pose = PoseStamped()

    def cancel_all_goals(self):
        loginfo('++ Waiting for the move base action server...')
        self.base_client.wait_for_server()
        loginfo('++ Canceling all goals on move base.')
        self.base_pending.clear()
        self.base_client.cancel_all_goals()
        sleep(3)

    def move_base(self, pose):
        """ Move base to a point of interest.

        :param :pose A PoseStamped point of interest.
        """

        roll, pitch, yaw = euler_from_quaternion([pose.orientation.x,
                                                  pose.orientation.y,
                                                  pose.orientation.z,
                                                  pose.orientation.w])
        transformend_orientation = quaternion_from_euler(roll, pitch, yaw + pi)
        pose.orientation.x = transformend_orientation[0]
        pose.orientation.y = transformend_orientation[1]
        pose.orientation.z = transformend_orientation[2]
        pose.orientation.w = transformend_orientation[3]

        pose.position.z = 0
        goal = MoveBaseGoal(target_pose=pose)

        loginfo('++ Waiting for move base action server...')
        self.base_client.wait_for_server()
        loginfo('++ Sending move base goal.')
        self.base_succeded.clear()
        self.base_pending.set()
        if self.verbose:
            loginfo(pose)
        self.base_client.send_goal(goal, feedback_db=self.base_feedback)
        loginfo('++ Waiting for response.')
        self.base_client.wait_for_result()
        status = self.base_client.get_state()
        verbose_status = ACTION_STATES[status]

        if status == GoalStatus.SUCCEEDED:
            loginfo('Move goal succeeded!')
            return True
        else:
            logerr('Move base goal failed with %', verbose_status)
            return False

    def base_feedback(self, pose):
        self.current_pose = pose
        if self.verbose:
            loginfo('++ Current pose updated.')
            loginfo(self.current_pose)

        # TODO Compare the distance between the goal and
        # the current pose and set an Event for convergence.
