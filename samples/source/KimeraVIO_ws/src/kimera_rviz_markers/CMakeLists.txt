cmake_minimum_required(VERSION 2.8.3)
project(kimera_rviz_markers)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin_simple REQUIRED)

catkin_simple()

cs_add_library(${PROJECT_NAME}
  src/utils.cpp
  src/${PROJECT_NAME}.cpp)

cs_add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

cs_install()

cs_export()
