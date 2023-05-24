cmake_minimum_required (VERSION 3.1)
project (cm_actuation)

set(CMAKE_CXX_STANDARD 11)

# find_package(AMBF)
find_package(Boost COMPONENTS program_options)
find_package(catkin COMPONENTS 
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



catkin_package(
 INCLUDE_DIRS .
 CATKIN_DEPENDS ambf_msgs ambf_client canopen roscpp
)

# add_executable(can_test code/can_test.cpp)
# target_link_libraries(can_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})