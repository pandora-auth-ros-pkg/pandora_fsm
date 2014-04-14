#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2014, P.A.N.D.O.R.A. Team.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of P.A.N.D.O.R.A. Team nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Chris Zalidis

import roslib
roslib.load_manifest('pandora_fsm')  
roslib.load_manifest('smach')
roslib.load_manifest('smach_ros')
import rospy
import smach
import smach_ros

import pandora_fsm

import threading

from smach import State, StateMachine

from pandora_fsm.states.navigation import *
from pandora_fsm.states.state_changer import *
from pandora_fsm.states.arena_identification import *
from pandora_fsm.containers.exploration import *
from pandora_fsm.containers.identification import *
from pandora_fsm.containers.data_fusion_hold import *
from pandora_fsm.containers.victim_validation import *

from state_manager_communications.msg import robotModeMsg

def main():
	
	rospy.init_node('fsm')
	
	sm = StateMachine(outcomes=['preempted'])
		
	with sm:
		
		sm.userdata.numberOfVictims = 0
		
		StateMachine.add(
			'MONITOR_START',
			MonitorModeState(robotModeMsg.MODE_START_AUTONOMOUS),
			transitions={
			'invalid':'MONITOR_START',
			'valid':'ALL_YELLOW',
			'preempted':'preempted'
			}
		)
		
		sm_yellow = StateMachine(outcomes=['preempted'],
															input_keys=['numberOfVictims'],
															output_keys=['numberOfVictims'])
		
		with sm_yellow:
		
			StateMachine.add(
				'WAIT_FOR_SLAM',
				Timer(10),
				transitions={
				'time_out':'INITIAL_TURN',
				'preempted':'preempted'
				}		
			)
			
			StateMachine.add(
				'INITIAL_TURN',
				InitialTurnState(), 
				transitions={
					'succeeded':'ROBOT_MODE_EXPLORATION',
					'aborted':'ROBOT_MODE_EXPLORATION',
					'preempted':'preempted'
				}
			)
			
			StateMachine.add(
				'ROBOT_MODE_EXPLORATION',
				ChangeRobotModeState(robotModeMsg.MODE_EXPLORATION),
				transitions={
					'succeeded':'EXPLORATION',
					'preempted':'preempted'
				}
			)
				
			StateMachine.add(
				'EXPLORATION',
				explorationWithVictims(),
				transitions={
				'camera_alert':'MONITOR_VICTIM_AND_DO_WORK',
				'thermal_alert':'MONITOR_VICTIM_AND_DO_WORK',
				'preempted':'preempted'
				}
			)
			
			#~ sm.userdata.victim_info = None
			
			sm_victim = StateMachine(outcomes=['verified','not_verified','preempted','aborted'],
																output_keys=['victim_info'])
			
			with sm_victim:
				
				sm_victim.userdata.victim_info = None
			
				StateMachine.add(
					'VICTIM_APPROACH',
					cameraIdentificationSimple(),
					transitions={
					'victim_approached':'ROBOT_MODE_DF_HOLD',
					'aborted':'aborted',
					'preempted':'preempted'
					}
				)
				
				
				StateMachine.add(
					'ROBOT_MODE_DF_HOLD',
					ChangeRobotModeState(robotModeMsg.MODE_DF_HOLD),
					transitions={
						'succeeded':'DATA_FUSION_HOLD',
						'preempted':'preempted'
					}
				)
				
				
				StateMachine.add(
					'DATA_FUSION_HOLD',
					dataFusionHold(),
					transitions={
					'verified':'verified',
					'not_verified':'not_verified',
					'preempted':'preempted'
					},
					remapping={'victim_info':'victim_info'}
				)
			
			
			cc = Concurrence(
				outcomes=[
					'verified',
					'not_verified',
					'update_victim',
					'preempted',
					'aborted'], 
				default_outcome = 'not_verified',
				output_keys=['victim_info'],
				outcome_map = {
					'verified':{'VICTIM_IDENTIFICATION':'verified','VICTIM_UPDATE':'preempted'},
					'not_verified':{'VICTIM_IDENTIFICATION':'not_verified','VICTIM_UPDATE':'preempted'},
					'preempted':{'VICTIM_IDENTIFICATION':'preempted','VICTIM_UPDATE':'preempted'}, 
					'aborted':{'VICTIM_IDENTIFICATION':'aborted','VICTIM_UPDATE':'preempted'},
					'update_victim':{'VICTIM_UPDATE':'update_victim','VICTIM_IDENTIFICATION':'preempted'}},
				child_termination_cb=_termination_cb)
				
			with cc:
				
				Concurrence.add('VICTIM_IDENTIFICATION', sm_victim, remapping={'victim_info':'victim_info'})
				
				Concurrence.add('VICTIM_UPDATE', MonitorVictimUpdateState())
			
			
			StateMachine.add(
				'MONITOR_VICTIM_AND_DO_WORK',
				cc,
				transitions={
				'verified':'VICTIM_VALIDATION',
				'not_verified':'MONITOR_VICTIM_AND_DO_WORK',
				'update_victim':'MONITOR_VICTIM_AND_DO_WORK',
				'preempted':'preempted',
				'aborted':'ROBOT_MODE_EXPLORATION'
				},
				remapping={'victim_info':'victim_info'}
			)
			
			StateMachine.add(
				'VICTIM_VALIDATION',
				validateVictim(),
				transitions={
				'valid':'MONITOR_VICTIM_AND_DO_WORK',
				'not_valid':'MONITOR_VICTIM_AND_DO_WORK',
				'preempted':'preempted'
				},
				remapping={'victim_info':'victim_info',
                    'numberOfVictims':'numberOfVictims'}
			)
		
		sm_orange = StateMachine(outcomes=['preempted'])
		
		sm_arena = StateMachine(outcomes=['yellow','orange','preempted'],
														input_keys=['numberOfVictims','arena_color'],
														output_keys=['numberOfVictims'])
		
		with sm_arena:
			
			StateMachine.add(
				'ARENA_IDENTIFICATION',
				MonitorArenaTypeState(),
				transitions={
					'yellow':'yellow',
					'orange':'CHECK_VICTIMS_FOUND',
					'yellow_black':'ARENA_IDENTIFICATION',
					'red':'ARENA_IDENTIFICATION',
					'invalid':'preempted',
					'preempted':'preempted'
				},
				remapping={'arena_color':'arena_color'}
			)
			
			StateMachine.add(
				'CHECK_VICTIMS_FOUND',
				CheckVictimsFoundState(),
				transitions={
					'victims_found':'orange',
					'no_victims_found':'FIND_FIRST_VICTIM',
					'preempted':'preempted'
				},
				remapping={'numberOfVictims':'numberOfVictims'}
			)
			
			StateMachine.add(
				'FIND_FIRST_VICTIM',
				FindFirstVictimState(),
				transitions={
					'succeeded':'orange',
					'preempted':'preempted'
				}
			)
		
		arena_color = 1
		
		con_yellow = Concurrence(
			outcomes=[
				'orange',
				'preempted',
				'shutdown'],
			default_outcome = 'preempted',
			input_keys=['numberOfVictims','arena_color'],
			output_keys=['numberOfVictims'],
			outcome_map = {
				'orange':{'ARENA_IDENTIFICATION':'orange', 'MONITOR_SHUTDOWN':'preempted', 'YELLOW':'preempted'},
				'preempted':{'YELLOW':'preempted', 'MONITOR_SHUTDOWN':'preempted', 'ARENA_IDENTIFICATION':'preempted'},
				'shutdown':{'MONITOR_SHUTDOWN':'valid','YELLOW':'preempted', 'ARENA_IDENTIFICATION':'preempted'}},
			child_termination_cb=_termination_cb_all)
		
		with con_yellow:
			
			Concurrence.add('YELLOW', sm_yellow, remapping={'numberOfVictims':'numberOfVictims'})
			
			Concurrence.add('MONITOR_SHUTDOWN', MonitorModeState(robotModeMsg.MODE_TELEOPERATED_LOCOMOTION))
			
			Concurrence.add('ARENA_IDENTIFICATION', sm_arena, remapping={'arena_color':'arena_color'})
		
		StateMachine.add(
			'ALL_YELLOW',
			con_yellow,
			transitions={
			'orange':'ALL_ORANGE',
			'shutdown':'MONITOR_START',
			'preempted':'preempted'
			},
			remapping={'numberOfVictims':'numberOfVictims',
									'arena_color':'arena_color'}
		)
		
		arena_color = 2
		
		con_yellow = Concurrence(
			outcomes=[
				'yellow',
				'preempted',
				'shutdown'],
			default_outcome = 'preempted',
			input_keys=['numberOfVictims','arena_color'],
			output_keys=['numberOfVictims'],
			outcome_map = {
				'yellow':{'ARENA_IDENTIFICATION':'yellow', 'MONITOR_SHUTDOWN':'preempted', 'ORANGE':'preempted'},
				'preempted':{'ORANGE':'preempted', 'MONITOR_SHUTDOWN':'preempted', 'ARENA_IDENTIFICATION':'preempted'},
				'shutdown':{'MONITOR_SHUTDOWN':'valid','ORANGE':'preempted', 'ARENA_IDENTIFICATION':'preempted'}},
			child_termination_cb=_termination_cb_all)
		
		with con_yellow:
			
			Concurrence.add('ORANGE', sm_orange, remapping={'numberOfVictims':'numberOfVictims'})
			
			Concurrence.add('MONITOR_SHUTDOWN', MonitorModeState(robotModeMsg.MODE_TELEOPERATED_LOCOMOTION))
			
			Concurrence.add('ARENA_IDENTIFICATION', sm_arena, remapping={'arena_color':'arena_color'})
		
		StateMachine.add(
			'ALL_ORANGE',
			con_yellow,
			transitions={
			'yellow':'ALL_YELLOW',
			'shutdown':'MONITOR_START',
			'preempted':'preempted'
			},
			remapping={'numberOfVictims':'numberOfVictims',
									'arena_color':'arena_color'}
		)
	
	sis = smach_ros.IntrospectionServer('fsm_introspection', sm, '/PANDORA_FSM')
	
	sis.start()
	
	smach_ros.set_preempt_handler(sm)
	
	# Execute SMACH tree in a separate thread so that we can ctrl-c the script
	smach_thread = threading.Thread(target = sm.execute)
	smach_thread.start()
	
	rospy.spin()
	sis.stop()


def _termination_cb(outcome_map):
	return True
	
def _termination_cb_all(outcome_map):
	return True
	

if __name__ == '__main__':
	main()
