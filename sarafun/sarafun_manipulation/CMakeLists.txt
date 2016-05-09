cmake_minimum_required(VERSION 2.8.3)
project(sarafun_manipulation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib_msgs
  actionlib
)

link_directories(${catkin_LIBRARY_DIRS})

add_action_files(
DIRECTORY action
FILES Approach.action Grab.action Place.action
)

generate_messages(
DEPENDENCIES actionlib_msgs std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(Approach src/Approach.cpp)
target_link_libraries(Approach ${catkin_LIBRARIES})

add_executable(Grab src/Grab.cpp)
target_link_libraries(Grab ${catkin_LIBRARIES})
