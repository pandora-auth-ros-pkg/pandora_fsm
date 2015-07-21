

""" Module with interfaces used by the Agent to communicate with
    the rest of the system.
"""

""" Data fusion """

# Action to delete a victim from the data fusion's registry.
delete_victim = '/data_fusion/delete_victim'

# Action to validate a victim.
validate_victim = '/data_fusion/validate_victim'

# Mark QRs as visited.
visit_qr = '/data_fusion/visit_qr'

# Service for the world model.
get_world_model = '/data_fusion/get_world_model'

# Action to notify data fusion about the current target.
choose_target = '/data_fusion/target_victim'

# Publishing QR notificatios from the data fusion's registry.
qr_notification = '/data_fusion/qr_notification'

# Holds the score for the robocup competition.
robocup_score = '/data_fusion/robocup_score'

# Keeps track of the covered area.
area_covered = '/data_fusion/sensor_coverage/area_covered'

# Used by data fusion to publish the world model.
world_model = '/data_fusion/world_model'


""" GUI """

# Action to wait for validation from the operator.
gui_validation = '/gui/validate_victim'

# Used to reset the robot.
robot_reset = '/gui/robot_reset'

# Used to restart the robot.
robot_restart = '/gui/robot_restart'

# Close the agent's process
destroy_agent = '/gui/destroy_agent'

""" Navigation """

# Action to communicate with the navigation node.
do_exploration = '/do_exploration'

# Subscriber on the navigation node.
# Possible arena types:
#   - Yellow         -> 0
#   - Orange         -> 1
#   - YellowAndBlack -> 2
#   - Red            -> 3
arena_type = '/navigation/arena_type'

# Moves the robot.
move_base = '/move_base'


""" Global state """

# Action to change the global state.
state_changer = '/robot/state/change'

# Monitors the number of clients registered in the State Manager.
state_monitor = '/robot/state/clients'

# Stops the current task.
agent_interrupt = '/agent/interrupt'


""" Control """

# Moves end effector to a point of interest.
move_end_effector_controller = '/control/move_end_effector_controller_action'
