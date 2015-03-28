""" Error handling with exceptions """


class MachineError(Exception):
    """ General Exception for the Machine """

    def __init__(self, value):
        self.value = value

    def __str__(self):
        return repr(self.value)
