from rospy import loginfo, sleep

from actionlib import SimpleActionClient as Client

from pandora_rqt_gui.msg import ValidateVictimGUIAction, ValidateVictimGUIGoal

from pandora_fsm import topics


class GUI(object):

    """ Communication with the operator."""

    def __init__(self, verbose=False):
        self.verbose = verbose
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
        goal.victimFoundx = target.VictimPose.pose.position.x
        goal.victimFoundy = target.VictimPose.pose.position.y
        goal.probability = target.probability
        goal.sensorIDsFound = target.sensors

        loginfo('^^ Waiting for the GUI action server.')
        self.client.wait_for_server()
        loginfo('^^ Sending validation request for: ')
        loginfo(target)
        self.client.send_goal(goal)
        loginfo('^^ Waiting for response.')
        self.client.wait_for_result()

        return self.client.get_result()
