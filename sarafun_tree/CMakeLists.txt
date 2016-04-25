cmake_minimum_required(VERSION 2.8.3)
project(sarafun_tree)

find_package(catkin REQUIRED COMPONENTS
  behavior_tree_leaves
  roscpp
  std_msgs
  actionlib_msgs
  actionlib
)

link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
  CATKIN_DEPENDS actionlib_msgs behavior_tree_leaves
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(GrabObjectAction src/GrabObjectAction.cpp)
target_link_libraries(GrabObjectAction ${catkin_LIBRARIES})
