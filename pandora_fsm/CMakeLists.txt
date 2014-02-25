cmake_minimum_required(VERSION 2.8.3)
project(pandora_fsm)
find_package(catkin REQUIRED COMPONENTS rospy 
										#~ smach 
										#~ smach_ros 
										actionlib 
										)

catkin_package(
    DEPENDS 
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

catkin_python_setup()
