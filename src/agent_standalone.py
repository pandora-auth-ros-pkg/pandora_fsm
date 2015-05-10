#!/usr/bin/env python

import sys
import rospy
from pandora_fsm import Agent

DEFAULT_STRATEGY = 'normal'


if __name__ == '__main__':
    rospy.init_node('agent_standalone')

    # Load a strategy from the terminal.
    if len(sys.argv) > 1:
        strategy = sys.argv[1]
        rospy.loginfo('Start with strategy -> %s', strategy)
    else:
        rospy.logwarn('No strategy was given. Using %s.', DEFAULT_STRATEGY)
        strategy = DEFAULT_STRATEGY

    agent = Agent(strategy=strategy)

    # Start the agent.
    agent.wake_up()
