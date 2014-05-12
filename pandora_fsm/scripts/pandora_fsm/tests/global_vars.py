#!/usr/bin/env python

import communications
from pandora_fsm.agent.agent_communications import *

test_agent = None
com = None

def init(unit):
  global test_agent
  test_agent = AgentCommunications()
  global com
  com = communications.Communications(unit)
