cmake_minimum_required (VERSION 3.1)
project (cm_actuation)

set(CMAKE_CXX_STANDARD 14)

add_subdirectory(code/canopen)

# find_package(AMBF)
find_package(Boost COMPONENTS program_options)
find_package(catkin COMPONENTS REQUIRED
ambf_msgs 
ambf_client 
rospy 
canopen
roscpp
)

# include_directories(${AMBF_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${AMBF_LIBRARY_DIRS})
add_definitions(${AMBF_DEFINITIONS})

add_library(cm_actuation SHARED 
code/BIGSSMaxonCAN.cpp
code/BIGSSMaxonCANROS.cpp
)
target_link_libraries(cm_actuation ${catkin_LIBRARIES} ${Boost_LIBRARIES})

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES cm_actuation
 CATKIN_DEPENDS ambf_msgs ambf_client canopen roscpp
)

include_directories(include)

add_executable(bigss_maxon_can_ros_node code/bigss_maxon_can_ros_node.cpp)
target_link_libraries(bigss_maxon_can_ros_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} cm_actuation)