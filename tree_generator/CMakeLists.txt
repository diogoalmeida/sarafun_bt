cmake_minimum_required(VERSION 2.8.3)
project(tree_generator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sarafun_msgs
  actionlib_msgs
  actionlib
)

link_directories(${catkin_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

catkin_package(
  CATKIN_DEPENDS actionlib actionlib_msgs sarafun_msgs roscpp
  INCLUDE_DIRS include
  LIBRARIES
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
