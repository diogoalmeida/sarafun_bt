cmake_minimum_required(VERSION 2.8.3)
project(sarafun_assembly)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib_msgs
  actionlib
)

link_directories(${catkin_LIBRARY_DIRS})

add_action_files(
DIRECTORY action
FILES Folding.action Insertion.action
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

add_executable(Folding src/Folding.cpp)
target_link_libraries(Folding ${catkin_LIBRARIES})

add_executable(Insertion src/Insertion.cpp)
target_link_libraries(Insertion ${catkin_LIBRARIES})
