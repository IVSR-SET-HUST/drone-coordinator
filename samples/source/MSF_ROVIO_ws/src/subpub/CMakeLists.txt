cmake_minimum_required(VERSION 3.0.2)
project(subpub)

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

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(target_pub src/target_pub.cpp)

target_link_libraries(target_pub
  ${catkin_LIBRARIES}
)
