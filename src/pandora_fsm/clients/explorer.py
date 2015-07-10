from threading import Event

from rospy import sleep

from geometry_msgs.msg import PoseStamped

from actionlib import GoalStatus
from actionlib import SimpleActionClient as Client

from pandora_exploration_msgs.msg import DoExplorationAction, DoExplorationGoal

from pandora_fsm import topics
from pandora_fsm.utils import logger as log
from pandora_fsm.utils import TERMINAL_STATES, ACTION_STATES


class Explorer(object):

    """
    Explorer client for the Agent. A wrapper API for easy communication
    and error handling for the exploration module.
    """

    def __init__(self, dispatcher, verbose=False):
        self.dispatcher = dispatcher
        self.client = Client(topics.do_exploration, DoExplorationAction)
        self.verbose = verbose
        self.exploration_pending = Event()
        self.pose_stamped = PoseStamped()

    def explore(self, exploration_type=DoExplorationGoal.TYPE_DEEP):

        goal = DoExplorationGoal(exploration_type=exploration_type)
        log.debug('Waiting for the Explorer action server...')
        self.client.wait_for_server()
        log.info('Sending new exploration goal.')
        self.exploration_pending.set()
        self.client.send_goal(goal, feedback_cb=self.exploration_feedback,
                              done_cb=self.exploration_done)

    def cancel_all_goals(self):
        log.debug('Waiting for the Explorer action server...')
        self.client.wait_for_server()
        log.info('Canceling all goals on Explorer.')
        self.exploration_pending.clear()
        self.client.cancel_all_goals()
        sleep(3)

    def exploration_feedback(self, pose_stamped):
        self.pose_stamped = pose_stamped
        if self.verbose:
            log.debug('Received feedback from Explorer:')
            log.info(self.pose_stamped)

    def exploration_done(self, status, result):

        # If the explorer is waiting on a goal.
        if self.exploration_pending.is_set():
            if status == GoalStatus.SUCCEEDED:
                log.warning('Exploration has finished.')
                self.exploration_pending.clear()
                self.dispatcher.emit('exploration.success')
            elif status in TERMINAL_STATES.keys():
                verbose_status = TERMINAL_STATES[status]
                log.error('Exploration has failed with %s.', verbose_status)
                self.exploration_pending.clear()
                self.dispatcher.emit('exploration.retry')

        if self.verbose:
            log.debug('Exploration goal status: %s', ACTION_STATES[status])
