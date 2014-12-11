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
# Author: Voulgarakis George <turbolapingr@gmail.com>

import roslib
roslib.load_manifest('pandora_fsm')
import rospy
import unittest
import communications

from pandora_fsm.robocup_agent.robocup_agent import RoboCupAgent
from state_manager_msgs.msg import RobotModeMsg, RobotModeGoal
from pandora_data_fusion_msgs.msg import VictimInfoMsg


class TestFSM(unittest.TestCase):

    def test_fsm(self):
        rospy.sleep(2.)
        msg = RobotModeMsg()
        msg.mode = msg.MODE_START_AUTONOMOUS
        msg.type = msg.TYPE_TRANSITION
        goal = RobotModeGoal(modeMsg=msg)
        com.state_changer_ac_.send_goal(goal)
        com.state_changer_ac_.wait_for_result()
        rospy.sleep(12.)

        victims = []
        victim = VictimInfoMsg()
        victim.id = 5
        victim.victimPose.pose.position.x = \
            com.robot_pose_.pose.position.x + 1.1
        victim.victimPose.pose.position.y = \
            com.robot_pose_.pose.position.y + 0.5
        victim.victimPose.pose.position.z = 0.4
        victim.probability = 0.0
        victims.append(victim)
        com.victims_pub_.publish(victims)
        rospy.sleep(3.)

        victims = []
        victim = VictimInfoMsg()
        victim.id = 5
        victim.victimPose.pose.position.x = \
            com.robot_pose_.pose.position.x + 1.0
        victim.victimPose.pose.position.y = \
            com.robot_pose_.pose.position.y + 0.5
        victim.victimPose.pose.position.z = 0.4
        victim.probability = 0.0
        victims.append(victim)
        com.victims_pub_.publish(victims)
        rospy.sleep(11.)

        victims = []
        victim = VictimInfoMsg()
        victim.id = 5
        victim.victimPose.pose.position.x = com.robot_pose_.pose.position.x
        victim.victimPose.pose.position.y = com.robot_pose_.pose.position.y
        victim.victimPose.pose.position.z = 0.4
        victim.probability = 0.8
        victims.append(victim)
        com.victims_pub_.publish(victims)
        rospy.sleep(10.)

        msg = RobotModeMsg()
        msg.mode = msg.MODE_TELEOPERATED_LOCOMOTION
        msg.type = msg.TYPE_TRANSITION
        goal = RobotModeGoal(modeMsg=msg)
        com.state_changer_ac_.send_goal(goal)
        com.state_changer_ac_.wait_for_result()
        rospy.sleep(1.)

if __name__ == '__main__':
    rospy.init_node('test_fsm')
    com = communications.Communications('fsm')
    test_state_client = communications.TestStateClient()
    rospy.sleep(1.)

    #test FSM
    robot_start_state = unittest.TestSuite()
    robot_start_state.addTest(TestFSM('test_fsm'))
    unittest.TextTestRunner(verbosity=1).run(robot_start_state)

    com.delete_action_servers()

    rospy.spin()
