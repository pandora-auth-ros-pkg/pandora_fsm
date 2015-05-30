import threading

from pandora_data_fusion_msgs.msg import VictimInfoMsg

import config as conf
from utils import logger as log
from utils import distance_2d


class Target(object):

    """ Agent's target. """

    def __init__(self, dispatcher):
        self.dispatcher = dispatcher

        self.info = VictimInfoMsg()
        self.verified = threading.Event()
        self.identified = threading.Event()
        self.is_empty = True
        log.debug('Target initialized.')

    def set(self, info):
        """
        Set a new point of interest.

        :param info: A VictimInfoMsg instance to mark as the point of interest.
        """
        if isinstance(info, VictimInfoMsg):
            self.info = info

            # Reset the target's state.
            self.verified.clear()
            self.identified.clear()
            self.is_empty = False

            id = self.info.id
            probability = self.info.probability
            log.info('Target acquired!')
            log.info('==> #%d with probability %.2f', id, probability)
        else:
            log.error('Called set on target with invalid info msg.')

    def update(self, targets):

        if self.is_empty:
            log.error('Called update on empty target.')
            return

        for target in targets:
            if target.id == self.info.id:
                self.info = target
                if self.info.probability > conf.VERIFICATION_THRESHOLD:
                    self.verified.set()
                    log.debug('Target #%d is verified with %.2f',
                              self.info.id, self.info.probability)
                if self.info.probability > conf.IDENTIFICATION_THRESHOLD:
                    log.debug('Target #%d is identified with %.2f',
                              self.info.id, self.info.probability)
                    self.identified.set()
                self.update_move_base_goal(target.victimPose,
                                           self.info.victimPose)

    def update_move_base_goal(self, new_pose, old_pose):
        """
        Updates the current MoveBase goal while the target's pose
        is updated.

        If the distance between the new and the old pose is big enough
        the agent is informed through the dispatcher to send a new
        MoveBase goal.

        :param new_pose: The target's new pose.
        :param old_pose: The target's current pose.
        """
        if distance_2d(old_pose.pose, new_pose.pose) > conf.BASE_THRESHOLD:
            self.dispatcher.emit('move_base.resend', new_pose.pose)

    def is_verified(self):
        """
        Return True if the probability of the target is greater
        than the VERIFICATION_THRESHOLD.
        """

        return self.verified.is_set()

    def is_identified(self):
        """
        Return True if the probability of the target is greater
        than the IDENTIFICATION_THRESHOLD.
        """

        return self.identified.is_set()

    def clean(self):
        """
        Reset the info.
        """
        if self.is_empty:
            return

        self.info = VictimInfoMsg()
        self.identified.clear()
        self.verified.clear()
        log.debug('Target has been reseted.')
