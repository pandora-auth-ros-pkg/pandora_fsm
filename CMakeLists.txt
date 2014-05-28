cmake_minimum_required(VERSION 2.8.3)
project(pandora_fsm)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
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
  INCLUDE_DIRS
  LIBRARIES
)
