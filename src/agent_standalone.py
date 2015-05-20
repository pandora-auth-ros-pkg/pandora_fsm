#!/usr/bin/env python

import rospy
from pandora_fsm import Agent

DEFAULT_STRATEGY = 'normal'


if __name__ == '__main__':
    rospy.init_node('agent_standalone')

    agent = Agent(strategy=DEFAULT_STRATEGY)

    # Start the agent.
    agent.wake_up()

    rospy.spin()
