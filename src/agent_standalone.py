#!/usr/bin/env python

import rospy
import sys
from pandora_fsm import Agent


if __name__ == '__main__':
    if len(sys.argv) > 1:
        strategy = sys.argv[1]
    else:
        strategy = 'normal'

    rospy.init_node('agent_standalone')
    agent = Agent(strategy=strategy)

    # Start the agent.
    agent.wake_up()

    rospy.spin()
