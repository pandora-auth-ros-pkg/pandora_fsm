cmake_minimum_required(VERSION 2.8.3)
project(pandora_fsm)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  genmsg
  actionlib_msgs
  actionlib
  roslint
  pandora_gui_msgs
)

catkin_python_setup()


generate_dynamic_reconfigure_options(
  config/params.cfg
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    actionlib
    dynamic_reconfigure
)

add_executable(victim_validator src/victim_validator.cpp)
target_link_libraries(
    victim_validator
    ${catkin_LIBRARIES}
)

add_dependencies(
    victim_validator
    ${catkin_EXPORTED_TARGETS}
    ${pandora_gui_msgs_EXPORTED_TARGETS}
)

FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_PYTHON
     ${PROJECT_SOURCE_DIR}
     src/**/*.py
     tests/**/*.py
    )
roslint_python(${${PROJECT_NAME}_LINT_PYTHON})

if (CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(tests/unit/world_model.py)
  catkin_add_nosetests(tests/unit/utilities.py)
  catkin_add_nosetests(tests/unit/fsm_framework.py)
endif()
