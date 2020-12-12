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
  roslaunch
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

roslaunch_add_file_check(launch)

add_executable(hovering src/hovering_node.cpp)
target_link_libraries(hovering
  ${catkin_LIBRARIES}
)

add_executable(offboard src/offboard_node.cpp)
target_link_libraries(offboard
  ${catkin_LIBRARIES}
)

add_executable(gps_offb src/gps_offb.cpp)
target_link_libraries(gps_offb
  ${catkin_LIBRARIES}
)

add_executable(setmode_offb src/setmode_offb.cpp)
target_link_libraries(setmode_offb
  ${catkin_LIBRARIES}
)