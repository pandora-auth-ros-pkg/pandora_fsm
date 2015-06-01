
"""
    Agent's parameters and configuration.
"""

# Probability limit for a target to be verified as victim.
VERIFICATION_THRESHOLD = 0.8

# Probability limit for a target to be identified as a potential victim.
IDENTIFICATION_THRESHOLD = 0.6

# Time needed for the sensors to verify the current target.
VERIFICATION_TIMEOUT = 15

# Time limit for a global state change.
STATE_CHANGE_TIMEOUT = 20

# Number of MoveBase failures before the agent aborts the current goal.
MOVE_BASE_RETRY_LIMIT = 3

# Time limit for the MoveBase to succeed at a given goal.
MOVE_BASE_TIMEOUT = 120

# Minimum radius to classify a pose as different from the current goal.
BASE_THRESHOLD = 0.2
