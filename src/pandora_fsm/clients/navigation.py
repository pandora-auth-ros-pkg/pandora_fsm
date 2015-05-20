from threading import Event

from rospy import logerr, loginfo, sleep

from geometry_msgs.msg import PoseStamped

from actionlib import GoalStatus
from actionlib import SimpleActionClient as Client

from pandora_navigation_msgs.msg import DoExplorationAction, DoExplorationGoal

from pandora_fsm import topics
from pandora_fsm.utils import TERMINAL_STATES, ACTION_STATES


class Navigation(object):

    """ Navigation client for the Agent. A wrapper API
        for easy communication and error handling for the
        Navigation module.
    """

    def __init__(self, dispatcher, verbose=False):
        self.dispatcher = dispatcher
        self.explorer = Client(topics.do_exploration, DoExplorationAction)
        self.verbose = verbose
        self.exploration_pending = Event()
        self.base_position = PoseStamped()

    def explore(self, exploration_type=DoExplorationGoal.TYPE_NORMAL):

        goal = DoExplorationGoal(exploration_type=exploration_type)
        loginfo('-- Waiting for the navigation action server...')
        self.explorer.wait_for_server()
        loginfo('-- Sending new exploration goal.')
        self.exploration_pending.set()
        self.explorer.send_goal(goal, feedback_cb=self.navigation_feedback,
                                done_cb=self.navigation_done)

    def cancel_all_goals(self):
        loginfo('-- Waiting for the navigation action server...')
        self.explorer.wait_for_server()
        loginfo('-- Canceling all goals on navigation.')
        self.exploration_pending.clear()
        self.explorer.cancel_all_goals()
        sleep(3)

    def navigation_feedback(self, pose):
        self.base_position = PoseStamped()
        if self.verbose:
            loginfo('-- Received feedback from navigation:')
            loginfo(self.base_position)

    def navigation_done(self, status, result):

        # If the explorer is waiting on a goal.
        if self.exploration_pending.is_set():
            if status == GoalStatus.SUCCEEDED:
                loginfo('-- Exploration has finished.')
                self.exploration_pending.clear()
                self.dispatcher.emit('exploration.success')
            elif status in TERMINAL_STATES.keys():
                verbose_status = TERMINAL_STATES[status]
                logerr('-- Exploration has failed with %s.', verbose_status)
                self.exploration_pending.clear()
                self.dispatcher.emit('exploration.retry')

        if self.verbose:
            loginfo('-- Exploration goal status: %s', ACTION_STATES[status])
