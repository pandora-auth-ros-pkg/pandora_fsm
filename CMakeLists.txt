cmake_minimum_required(VERSION 2.8.3)
project(pandora_fsm)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roslint
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  config/params.cfg
)

catkin_package(
  DEPENDS 
  CATKIN_DEPENDS
    rospy
    actionlib
    dynamic_reconfigure
    roslint
  INCLUDE_DIRS
  LIBRARIES
)

FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_PYTHON
     ${PROJECT_SOURCE_DIR} 
     src/pandora_fsm/containers/data_fusion_hold.py
     src/pandora_fsm/containers/exploration.py
     src/pandora_fsm/containers/identification.py
     src/pandora_fsm/containers/robot_start.py
     src/pandora_fsm/containers/validation.py
     src/pandora_fsm/robocup_agent/agent.py
     src/pandora_fsm/robocup_agent/agent_topics.py
     src/pandora_fsm/robocup_agent/cost_function.py
     src/pandora_fsm/robocup_agent/robocup_agent.py
     src/pandora_fsm/robocup_agent/robocup_cost_functions.py
     src/pandora_fsm/robocup_agent/robocup_states.py
     src/pandora_fsm/robocup_agent/state.py
     src/pandora_fsm/robocup_fsm/robocup_fsm.py
     src/pandora_fsm/states/my_monitor_state.py
     src/pandora_fsm/states/my_simple_action_state.py
     src/pandora_fsm/states/navigation.py
     src/pandora_fsm/states/state_changer.py
     src/pandora_fsm/states/victims.py
     tests/agent_functional_tests.py
     tests/agent_unit_tests.py
     tests/communications.py
     tests/global_vars.py
    )
roslint_python(${${PROJECT_NAME}_LINT_PYTHON})
