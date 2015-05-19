from rospy import loginfo, sleep, logerr

from actionlib import SimpleActionClient as Client
from actionlib import GoalStatus

from pandora_rqt_gui.msg import ValidateVictimGUIAction, ValidateVictimGUIGoal

from pandora_fsm import topics
from pandora_fsm.utils import ACTION_STATES


class GUI(object):

    """ Communication with the operator."""

    def __init__(self, verbose=False):
        self.verbose = verbose
        self.gui_result = None
        self.client = Client(topics.gui_validation, ValidateVictimGUIAction)

    def cancel_all_goals(self):
        loginfo('$$ Waiting for the GUI action server...')
        self.client.wait_for_server()
        loginfo('$$ Canceling all goals on GUI.')
        self.client.cancel_all_goals()
        sleep(3)

    def send_request(self, target):
        """ Sends a validation request to the robot operator.

        :param :target A target to be validated.
        """
        goal = ValidateVictimGUIGoal()
        goal.victimFoundx = target.victimPose.pose.position.x
        goal.victimFoundy = target.victimPose.pose.position.y
        goal.probability = target.probability
        goal.sensorIDsFound = target.sensors

        loginfo('^^ Waiting for the GUI action server.')
        self.client.wait_for_server()
        loginfo('^^ Sending validation request.')
        if self.verbose:
            loginfo(target)
        self.client.send_goal(goal)
        loginfo('^^ Waiting for response.')
        self.client.wait_for_result()

        status = self.client.get_state()
        verbose_status = ACTION_STATES[status]
        if status == GoalStatus.SUCCEEDED:
            loginfo('^^ Validation request succeded!')
            self.gui_result = self.client.get_result()
            return True
        else:
            logerr('^^ Validation request failed with %s.', verbose_status)
            return False

    def result(self):
        return self.gui_result
