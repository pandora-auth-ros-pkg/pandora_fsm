from actionlib import GoalStatus
from actionlib import SimpleActionClient as Client

from pandora_data_fusion_msgs.msg import DeleteVictimAction, ValidateVictimGoal
from pandora_data_fusion_msgs.msg import ValidateVictimAction, DeleteVictimGoal

from pandora_fsm import topics
from pandora_fsm.utils import ACTION_STATES
from pandora_fsm.utils import logger as log


class DataFusion(object):

    """ Data fusion client. Updates the data fusion registry after
        identification or validation from the operator.
    """

    def __init__(self, verbose=False):
        self.verbose = verbose
        self.deletion = Client(topics.delete_victim, DeleteVictimAction)
        self.validation = Client(topics.validate_victim, ValidateVictimAction)

    def delete_victim(self, victim_id):
        """ Deletes a victim for the data fusion registry.

        :param :victim_id The ID of the victim to be deleted.
        """
        goal = DeleteVictimGoal(victimId=victim_id)

        log.debug('Waiting for the DataFusion action server...')
        self.deletion.wait_for_server()
        log.warning('Deleting victim wth ID -> %d.', victim_id)
        self.deletion.send_goal(goal)
        log.debug('Waiting for response...')
        self.deletion.wait_for_result()

        status = self.deletion.get_state()
        verbose_status = ACTION_STATES[status]
        if status == GoalStatus.SUCCEEDED:
            log.info('Victim deletion succeded!')
            return True
        else:
            log.error('Victim deletion failed with %s.', verbose_status)
            return False

    def validate_victim(self, victim_id, valid=False):
        """ Updates the data fusion registry with valid or not victims.

        :param :victim_id The number of the victim's ID.
        :param :valid True if it was validated from the operator.
        """
        goal = ValidateVictimGoal()
        goal.victimId = victim_id
        goal.victimValid = valid
        victim_status = 'valid' if valid else 'not valid'

        log.debug('Waiting for the DataFusion action server...')
        self.validation.wait_for_server()
        log.info('Validating victim #%d as %s.', victim_id, victim_status)
        self.validation.send_goal(goal)
        log.debug('Waiting for response...')
        self.validation.wait_for_result()
        status = self.validation.get_state()
        verbose_status = ACTION_STATES[status]
        if status == GoalStatus.SUCCEEDED:
            log.info('Victim %d validated succesfully', victim_id)
            return True
        else:
            log.error('Validation failure with %s.', verbose_status)
            return False
