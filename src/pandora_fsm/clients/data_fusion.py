from actionlib import SimpleActionClient as Client
from actionlib import GoalStatus

from pandora_data_fusion_msgs.msg import (ChooseVictimAction, ChooseVictimGoal,
                                          ValidateVictimAction,
                                          ValidateVictimGoal)
from pandora_fsm import topics
from pandora_fsm.utils import logger as log
from pandora_fsm.utils import ACTION_STATES


class DataFusion(object):

    """
    Data fusion client. Updates the data fusion registry after
    identification or validation from the operator.
    """

    def __init__(self, verbose=False):
        self.verbose = verbose
        self.deletion = Client(topics.delete_victim, ChooseVictimAction)
        self.validation = Client(topics.validate_victim, ValidateVictimAction)
        self.selection = Client(topics.choose_target, ChooseVictimAction)

    def delete_victim(self, victim_id):
        """
        Delete a victim from the data fusion registry.

        :param :victim_id The ID of the victim to be deleted.
        """
        goal = ChooseVictimGoal(victimId=victim_id)

        log.debug('Waiting for the DataFusion action server...')
        self.deletion.wait_for_server()
        log.warning('Deleting victim wth ID -> %d.', victim_id)
        self.deletion.send_goal(goal)
        log.debug('Waiting for response...')
        self.deletion.wait_for_result()

        status = self.deletion.get_state()
        result = self.deletion.get_result()
        verbose_status = ACTION_STATES[status]

        if status == GoalStatus.SUCCEEDED:
            log.info('Victim deletion succeded!')
        else:
            log.error('Victim deletion failed with %s.', verbose_status)

        return result.worldModel

    def validate_victim(self, victim_id, valid=False, verified=False):
        """
        Update the data fusion registry with valid or not victims.

        :param :victim_id The victim's ID.
        :param :valid True if it was validated from the operator.
        :param :verified True if got into the sensor hold state.
        """
        goal = ValidateVictimGoal()
        goal.victimId = victim_id
        goal.victimValid = valid
        goal.victimVerified = verified

        victim_status = self.classify_target(valid, verified)

        log.debug('Waiting for the DataFusion action server...')
        self.validation.wait_for_server()
        log.info('Validating victim #%d as %s.', victim_id, victim_status)
        self.validation.send_goal(goal)
        log.debug('Waiting for response...')
        self.validation.wait_for_result()

        status = self.validation.get_state()
        result = self.validation.get_result()
        verbose_status = ACTION_STATES[status]

        if status == GoalStatus.SUCCEEDED:
            log.info('Victim %d validated successfully.', victim_id)
        else:
            log.error('Validation failure with %s.', verbose_status)

        return result.worldModel

    def announce_target(self, victim_id):
        """
        Notify data fusion about the current target.

        :param victim_id: The id of the current target/victim.
        """
        goal = ChooseVictimGoal(victimId=victim_id)

        log.debug('Waiting for the DataFusion action server...')
        self.selection.wait_for_server()
        log.info('Sending current target #%d', victim_id)
        self.selection.send_goal(goal)
        log.debug('Waiting for response...')
        self.selection.wait_for_result()

        status = self.selection.get_state()
        result = self.selection.get_result()
        verbose_status = ACTION_STATES[status]

        if status == GoalStatus.SUCCEEDED:
            log.info('Victim #%d, selected as the current target', victim_id)
        else:
            log.error('Victim selection failed with %s.', verbose_status)

        return result.worldModel

    def classify_target(self, valid=False, verified=False):
        """
        Characterize the target as true negative, false negative,
        false positive or true positive.

        :param valid: Response from the operator.
        :param valid: Robot's prediction.
        """
        if not valid and not verified:
            return 'TRUE NEGATIVE'
        elif valid and verified:
            return 'TRUE POSITIVE'
        elif not valid and verified:
            return 'FALSE POSITIVE'
        elif valid and not verified:
            return 'FALSE NEGATIVE'
