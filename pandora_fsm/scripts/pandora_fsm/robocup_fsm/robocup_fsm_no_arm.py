#!/usr/bin/env python
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
from pandora_fsm.containers.exploration import *
from pandora_fsm.containers.identification import *
from pandora_fsm.containers.data_fusion_hold import *
from pandora_fsm.containers.victim_validation import *

from state_manager_communications.msg import robotModeMsg

def main():
	
	rospy.init_node('fsm')
	
	sm = StateMachine(outcomes=['preempted'])
		
	with sm:
		
		StateMachine.add(
			'MONITOR_START',
			MonitorModeState(robotModeMsg.MODE_START_AUTONOMOUS),
			transitions={
			'invalid':'MONITOR_START',
			'valid':'ALL',
			'preempted':'preempted'
			}
		)
		
		sm_everything = StateMachine(outcomes=['preempted'])
		
		with sm_everything:
		
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
				'preempted':'preempted'}
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
			
			sm_victim = StateMachine(outcomes=['verified','not_verified','preempted','aborted'],output_keys=['victim_info'])
			
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
				
				Concurrence.add('VICTIM_IDENTIFICATION', sm_victim,remapping={'victim_info':'victim_info'})
				
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
				remapping={'victim_info':'victim_info'}
			)
			
		
		cc = Concurrence(
				outcomes=[
					'preempted',
					'shutdown'], 
				default_outcome = 'preempted',
				outcome_map = {
					'preempted':{'AUTONOMOUS':'preempted', 'MONITOR_SHUTDOWN':'preempted'},
					'shutdown':{'MONITOR_SHUTDOWN':'valid','AUTONOMOUS':'preempted'}},
				child_termination_cb=_termination_cb_all)
				
		with cc:
				
				Concurrence.add('AUTONOMOUS', sm_everything)
				
				Concurrence.add('MONITOR_SHUTDOWN', MonitorModeState(robotModeMsg.MODE_TELEOPERATED_LOCOMOTION))
				
		
		StateMachine.add(
				'ALL',
				cc,
				transitions={
				'shutdown':'MONITOR_START',
				'preempted':'preempted'
				}
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
