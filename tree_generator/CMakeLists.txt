cmake_minimum_required(VERSION 2.8.3)
project(tree_generator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  sarafun_msgs
  actionlib_msgs
  actionlib
)

link_directories(${catkin_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

catkin_package(
  CATKIN_DEPENDS actionlib actionlib_msgs sarafun_msgs roscpp roslib
  INCLUDE_DIRS include
  LIBRARIES
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(tree_generator src/TreeFromKF.cpp src/tree_generator.cpp)
target_link_libraries(tree_generator ${catkin_LIBRARIES})
add_dependencies(tree_generator ${catkin_EXPORTED_TARGETS})
