import threading

from pandora_data_fusion_msgs.msg import VictimInfoMsg

from utils import logger as log


class Target(object):

    """ Agent's target. """

    def __init__(self, verification_threshold, identification_threshold):
        self.info = VictimInfoMsg()
        self.verified = threading.Event()
        self.identified = threading.Event()
        self.verification_threshold = verification_threshold
        self.identification_threshold = identification_threshold
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
                if self.info.probability > self.verification_threshold:
                    self.verified.set()
                    log.debug('Target #%d is verified with %.2f',
                              self.info.id, self.info.probability)
                if self.info.probability > self.identification_threshold:
                    log.debug('Target #%d is identified with %.2f',
                              self.info.id, self.info.probability)
                    self.identified.set()

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
        log.debug('Target has been reseted.')
