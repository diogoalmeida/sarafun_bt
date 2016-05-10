cmake_minimum_required(VERSION 2.8.3)
project(sarafun_generic_al_server)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib_msgs
  actionlib
)

link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)
