#!/usr/bin/env python

from rospy import init_node, spin

from pandora_data_fusion_msgs.msg import (ChooseVictimAction,
                                          ValidateVictimAction)
from pandora_end_effector_controller.msg import MoveEndEffectorAction
from pandora_exploration_msgs.msg import DoExplorationAction

from pandora_fsm import topics
from pandora_fsm.mocks import (MockActionServer, MoveBaseServer,
                               DataFusionServer)
from pandora_gui_msgs.msg import ValidateVictimGUIAction
from pandora_data_fusion_msgs.msg import (ChooseVictimAction,
                                          ChooseVictimResult,
                                          ValidateVictimAction,
                                          ValidateVictimResult)

if __name__ == '__main__':
    init_node('mock_node')

    # Action Servers
    # MoveBaseServer('move_base', topics.move_base)

    # DataFusionServer('choose_target', topics.choose_target, ChooseVictimAction, ChooseVictimResult)
    # DataFusionServer('delete_victim', topics.delete_victim, ChooseVictimAction, ChooseVictimResult)
    # DataFusionServer('fusion_validate', topics.validate_victim, ValidateVictimAction, ValidateVictimResult)

    MockActionServer('effector', topics.move_end_effector_controller, MoveEndEffectorAction)
    # MockActionServer('explorer', topics.do_exploration, DoExplorationAction)
    MockActionServer('validate_gui', topics.gui_validation, ValidateVictimGUIAction)

    spin()
