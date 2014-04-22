#!/usr/bin/env python
import roslib; roslib.load_manifest('pandora_fsm')
import rospy
import smach
import smach_ros

from smach import State, StateMachine

from pandora_fsm.states.my_monitor_state import MyMonitorState
from pandora_fsm.states.my_simple_action_state import MySimpleActionState
from fsm_communications.msg import *

from actionlib import *
from actionlib.msg import *
#import ArenaTypeMsg
#import FindFirstVictimAction

arena_type_topic = '/arena_type'
find_first_victim_topic = '/first_victim'

class MonitorArenaTypeState(MyMonitorState):
  
  def __init__(self):
    MyMonitorState.__init__(self, arena_type_topic,
                            None, #ArenaTypeMsg.msg
                            self.identify_cb,
                            extra_outcomes=['yellow','orange',
                                            'yellow_black','red'],
                            in_keys=['arena_color'])
  
  def execute(self, userdata):
    current_arena = userdata.arena_color
  
  def identify_cb(self, userdata, msg):
    rospy.sleep(5)
    
    #~ if msg.arenaType == current_arena:
      #~ return None
    #~ else:
      #~ if msg.arenaType == msg.TYPE_YELLOW:
        #~ return 'yellow'
      #~ elif msg.arenaType == msg.TYPE_ORANGE:
        #~ return 'orange'
      #~ elif msg.arenaType == msg.TYPE_YELLOW_BLACK:
        #~ return 'yellow_black'
      #~ elif msg.arenaType == msg.TYPE_RED:
        #~ return 'red'
      #~ else:
    return 'preempted'

class CheckVictimsFoundState(State):
  
  def __init__(self):
    State.__init__(self, outcomes=['victims_found','no_victims_found',
                                    'preempted'],
                    input_keys=['numberOfVictims'])
    
  def execute(self, userdata):
    if userdata.numberOfVictims > 0:
      return 'victims_found'
    elif userdata.numberOfVictims == 0:
      return 'no_victims_found'
    else:
      return 'preempted'

class FindFirstVictimState(MySimpleActionState):
  
  def __init__(self):
    MySimpleActionState.__init__(self, find_first_victim_topic,
                                  ValidateVictimAction, #FindFirstVictimAction,
                                  outcomes=['succeeded','preempted'],
                                  #~ result_cb=first_victim_found
                                )
  
  #~ def first_victim_found(self, userdata, status, result):
    #~ if status == GoalStatus.SUCCEEDED:
      #~ if result.victimFound:
        #~ return 'succeded'
      #~ else:
        #~ return 'preempted'
