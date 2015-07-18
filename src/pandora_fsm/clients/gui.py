from time import sleep

from actionlib import SimpleActionClient as Client
from actionlib import GoalStatus

from pandora_gui_msgs.msg import ValidateVictimGUIAction, ValidateVictimGUIGoal

from pandora_fsm import topics
from pandora_fsm.utils import ACTION_STATES
from pandora_fsm.utils import logger as log


class GUI(object):

    """ Communication with the operator."""

    def __init__(self, verbose=False):
        self.verbose = verbose
        self.gui_result = None
        self.client = Client(topics.gui_validation, ValidateVictimGUIAction)

    def cancel_all_goals(self):
        log.debug('Waiting for the GUI action server...')
        self.client.wait_for_server()
        log.info('Canceling all goals on GUI.')
        self.client.cancel_all_goals()
        sleep(3)

    def send_request(self, target):
        """ Sends a validation request to the robot operator.

        :param :target A target to be validated.
        """
        goal = ValidateVictimGUIGoal()
        goal.victimId = target.id
        goal.victimFoundx = target.victimPose.pose.position.x
        goal.victimFoundy = target.victimPose.pose.position.y
        goal.probability = target.probability
        goal.sensorIDsFound = target.sensors

        log.debug('Waiting for the GUI action server.')
        self.client.wait_for_server()
        log.info('Sending validation request.')
        if self.verbose:
            log.debug(target)
        self.client.send_goal(goal)
        log.info('Waiting for response.')
        self.client.wait_for_result()

        status = self.client.get_state()
        verbose_status = ACTION_STATES[status]
        if status == GoalStatus.SUCCEEDED:
            log.info('Validation request succeded!')
            self.gui_result = self.client.get_result()
            return True
        else:
            log.error('Validation request failed with %s.', verbose_status)
            return False

    def result(self):
        return self.gui_result
