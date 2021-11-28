cmake_minimum_required(VERSION 2.8)
project(disparity_image_proc)

find_package(catkin REQUIRED
  cv_bridge
  dynamic_reconfigure
  image_geometry
  image_proc
  image_transport
  message_filters
  nodelet
  sensor_msgs
  stereo_msgs)
find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
  CATKIN_DEPENDS image_geometry image_proc sensor_msgs stereo_msgs
  LIBRARIES ${PROJECT_NAME}
)

find_package(OpenCV REQUIRED calib3d)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS}
                           ${OpenCV_INCLUDE_DIRS}
)

# Nodelet library
add_library(${PROJECT_NAME} src/nodelets/depth_image.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
