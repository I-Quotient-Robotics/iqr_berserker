cmake_minimum_required(VERSION 2.8.3)
project(berserker_bringup)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package()

#include_directories(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)  

include_directories(
	include
  ${catkin_INCLUDE_DIRS}
)



