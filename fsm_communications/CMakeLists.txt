cmake_minimum_required(VERSION 2.8.3)
project(fsm_communications)
find_package(catkin REQUIRED COMPONENTS message_generation actionlib_msgs )

add_action_files(
  FILES
  ValidateVictim.action
  )

generate_messages(
  DEPENDENCIES actionlib_msgs
)
catkin_package(
  CATKIN_DEPENDS message_runtime 
)
