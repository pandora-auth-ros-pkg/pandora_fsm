#!/usr/bin/env python

import roslib
roslib.load_manifest('pandora_fsm')
import rospy
import state_manager
import dynamic_reconfigure.server
import topics

def boot(self):
	goal = MoveEndEffectorGoal(command=MoveEndEffectorGoal.TEST)
    self.end_effector_planner_ac_.send_goal_and_wait(goal)
    self.end_effector_planner_ac_.wait_for_result()
	if self.end_effector_planner_ac_.get_state() == GoalStatus.ABORTED:
		self.to_boot()
		rospy.loginfo("failed to test end effector")
	goal = MoveEndEffectorGoal(command=MoveEndEffectorGoal.PARK)
    self.end_effector_planner_ac_.send_goal_and_wait(goal)
	self.end_effector_planner_ac_.wait_for_result()
	if self.end_effector_planner_ac_.get_state() == GoalStatus.ABORTED:
		self.to_boot()
		rospy.loginfo("failed to park end effector")
	else 
		self.booted()


def scan(self):
	self.preempt_end_effector_planner()
	goal = MoveEndEffectorGoal(command=MoveEndEffectorGoal.SCAN)
	self.end_effector_planner_ac_.send_goal(goal)
	#previous implementation didn't wait for a result
	self.

def preempt_end_effector_planner(self):
	self.end_effector_planner_ac_.cancel_all_goals()
        


def explore(self):
	goal = DoExplorationGoal(exploration_type = self.current_exploration_mode)
	self.do_exploration_ac_.send_goal(goal, feedback_cb=self.feedback_cb,
									  done_cb=self.done_cb)
 
                    
