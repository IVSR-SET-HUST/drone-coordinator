cmake_minimum_required(VERSION 2.8.3)
project(dbow2_catkin)

find_package(catkin_simple REQUIRED)
catkin_simple()

include(ExternalProject)

file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

ExternalProject_Add(dbow2_src
  GIT_REPOSITORY https://github.com/dorian3d/DBoW2.git
  GIT_TAG master
  UPDATE_COMMAND ""
  #PATCH_COMMAND ""
  CONFIGURE_COMMAND cd ../dbow2_src && cmake -DCMAKE_BUILD_TYPE=Release
                                             -DCMAKE_INSTALL_PREFIX:PATH=${CATKIN_DEVEL_PREFIX}
                                             -DBUILD_Demo=Off .
  BUILD_COMMAND cd ../dbow2_src && make
  INSTALL_COMMAND cd ../dbow2_src && make install
)

install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/DBoW2
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h")

install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        FILES_MATCHING PATTERN "libDBoW2*")

cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include
          LIBRARIES DBoW2)
