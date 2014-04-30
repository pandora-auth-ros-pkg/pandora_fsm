cmake_minimum_required(VERSION 2.8.3)
project(fsm_communications)
find_package(catkin REQUIRED COMPONENTS message_generation actionlib_msgs std_msgs)

add_message_files(
  FILES
  ArenaTypeMsg.msg
)

add_action_files(
  FILES
  ValidateVictim.action
  AgentCommunication.action
  RobotStarted.action
  ExplorationEnded.action
  MonitorVictimEnded.action
  ValidateVictimEnded.action
  AbortFSM.action
  ForceFSMRestart.action
)

generate_messages(
  DEPENDENCIES std_msgs actionlib_msgs
)
catkin_package(
  CATKIN_DEPENDS message_runtime actionlib_msgs std_msgs
)
