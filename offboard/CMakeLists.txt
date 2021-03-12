cmake_minimum_required(VERSION 3.0.2)
project(offboard)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  rospy
  std_msgs
)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES offboard
   CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp rospy std_msgs 
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(
  offboard_lib
  src/offboard_lib.cpp
)
add_library(
  logging_lib
  src/logging_lib.cpp
)

target_link_libraries(offboard_lib
  ${catkin_LIBRARIES}
)
target_link_libraries(logging_lib
  ${catkin_LIBRARIES}
)

add_executable(offboard src/offboard_node.cpp)
add_executable(hover src/hover_node.cpp)
add_executable(logging src/logging_node.cpp)
add_executable(setmode_offb src/setmode_offb.cpp)

target_link_libraries(offboard
  offboard_lib
  logging_lib
)
target_link_libraries(hover
  offboard_lib
  logging_lib
)
target_link_libraries(logging
  offboard_lib
  logging_lib
)
target_link_libraries(setmode_offb
  offboard_lib
  logging_lib
)
