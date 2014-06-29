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
     src/pandora_fsm/agent_cost_functions/*.py
     src/pandora_fsm/agent_states/*.py
     src/pandora_fsm/fsm_containers/*.py
     src/pandora_fsm/fsm_states/*.py
     src/pandora_fsm/robocup_agent/*.py
     src/pandora_fsm/robocup_fsm/*.py
     tests/*.py
    )
roslint_python(${${PROJECT_NAME}_LINT_PYTHON})
