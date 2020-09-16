cmake_minimum_required(VERSION 3.0.2)
project(offboard)

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

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/offboard.cpp
# )

## Add executable for takeoff_node
#add_executable(offboard src/offboard_node.cpp)
add_executable(hovering src/hovering_node.cpp)

# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# target_link_libraries(offboard
#  ${catkin_LIBRARIES}
#)
target_link_libraries(hovering
  ${catkin_LIBRARIES}
)
