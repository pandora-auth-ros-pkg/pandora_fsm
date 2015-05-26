#!/usr/bin/env python

from rospy import init_node, spin

from pandora_fsm import topics
from pandora_fsm.mocks import MoveBaseServer, MockActionServer

from pandora_data_fusion_msgs.msg import DeleteVictimAction
from pandora_data_fusion_msgs.msg import ValidateVictimAction

from pandora_rqt_gui.msg import ValidateVictimGUIAction

from pandora_exploration_msgs.msg import DoExplorationAction
from pandora_end_effector_controller.msg import MoveEndEffectorAction


if __name__ == '__main__':
    init_node('mock_node')

    # Action Servers
    MoveBaseServer('move_base', topics.move_base)
    MockActionServer('effector', topics.move_end_effector_controller,
                     MoveEndEffectorAction)
    MockActionServer('explorer', topics.do_exploration, DoExplorationAction)
    MockActionServer('validate_gui', topics.gui_validation,
                     ValidateVictimGUIAction)
    MockActionServer('delete_victim', topics.delete_victim, DeleteVictimAction)
    MockActionServer('fusion_validate', topics.validate_victim,
                     ValidateVictimAction)
    spin()
