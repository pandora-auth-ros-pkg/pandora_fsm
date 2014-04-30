#!/usr/bin/env python

import roslib; roslib.load_manifest('pandora_fsm')
import rospy
import smach
import smach_ros
import pandora_fsm

from pandora_fsm.states.state_changer import ChangeRobotModeState
from pandora_fsm.states.my_monitor_state import MyMonitorState

from state_manager_communications.msg import robotModeMsg
from std_msgs.msg import Empty

from actionlib import *
from actionlib.msg import *

robot_start_topic = '/fsm/robot_start'
exploration_start_topic = '/fsm/exploration_start'
monitor_victim_start_topic = '/fsm/monitor_victim_start'
validate_victim_start_topic = '/fsm/validate_victim_start'
abort_fsm_topic = '/fsm/abort_fsm'

class RobotStart(MyMonitorState):
  
  def __init__(self):
    MyMonitorState.__init__(self, robot_start_topic, Empty, self.monitor_cb,
                            extra_outcomes=['succeeded'])
  
  def monitor_cb(self, userdata, msg):
    return 'succeeded'

class ExplorationStart(MyMonitorState):
  
  def __init__(self):
    MyMonitorState.__init__(self, exploration_start_topic, Empty,
                            self.monitor_cb, extra_outcomes=['succeeded'])
  
  def monitor_cb(self, userdata, msg):
    ChangeRobotModeState(robotModeMsg.MODE_EXPLORATION)
    return 'succeeded'

class MonitorVictimStart(MyMonitorState):
  
  def __init__(self):
    MyMonitorState.__init__(self, monitor_victim_start_topic, Empty,
                            self.monitor_cb, extra_outcomes=['succeeded'])
  
  def monitor_cb(self, userdata, msg):
    return 'succeeded'

class ValidateVictimStart(MyMonitorState):
  
  def __init__(self):
    MyMonitorState.__init__(self, validate_victim_start_topic, Empty,
                            self.monitor_cb, extra_outcomes=['succeeded'])
  
  def monitor_cb(self, userdata, msg):
    return 'succeeded'

class AbortFSM(MyMonitorState):
  
  def __init__(self):
    MyMonitorState.__init__(self, abort_fsm_topic, Empty, self.monitor_cb,
                            extra_outcomes=['aborted'])
  
  def monitor_cb(self, userdata, msg):
    return 'aborted'
