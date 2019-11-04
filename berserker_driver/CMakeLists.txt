cmake_minimum_required(VERSION 2.8.3)
project(berserker_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES berserker_driver
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(BerserkerDriverNode
  src/BerserkerDriverNode.cpp
  src/BerserkerDriver.cpp
  src/QThread.cpp
  src/QSerialPort.cpp
  src/QSerialFrame.cpp
)  
target_link_libraries(BerserkerDriverNode
  ${catkin_LIBRARIES}
)

