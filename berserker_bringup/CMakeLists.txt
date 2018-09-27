cmake_minimum_required(VERSION 2.8.3)
project(berserker_bringup)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
	nav_msgs
	sensor_msgs
  tf	
  message_generation  
  dynamic_reconfigure 
)



catkin_package()

#include_directories(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)  

include_directories(
	include
  ${catkin_INCLUDE_DIRS}
)


add_executable(berserker_driver 
  nodes/berserker_driver_node.cpp
  src/CRobotSerial.cpp
  src/CSerial.cpp
  src/function.cpp
  src/CUart.cpp
  src/CRobot.cpp
  src/thread.cpp
)  
target_link_libraries(berserker_driver 
  ${catkin_LIBRARIES}
)

