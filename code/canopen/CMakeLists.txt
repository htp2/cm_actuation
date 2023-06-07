cmake_minimum_required(VERSION 2.8.3)
project(canopen)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp)

add_library(canopen src/canframe.cpp src/canopen.cpp src/socketcan.cpp )

catkin_package( INCLUDE_DIRS include LIBRARIES canopen )

include_directories( ${catkin_INCLUDE_DIRS} include )

install(
  DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION} )

