cmake_minimum_required(VERSION 2.8.3)
project(sarafun_assembly)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sarafun_generic_al_server
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

add_executable(FoldingDummy src/FoldingDummy.cpp)
target_link_libraries(FoldingDummy ${catkin_LIBRARIES})
add_dependencies(FoldingDummy ${sarafun_assembly_EXPORTED_TARGETS})

add_executable(InsertionDummy src/InsertionDummy.cpp)
target_link_libraries(InsertionDummy ${catkin_LIBRARIES})
add_dependencies(InsertionDummy ${sarafun_assembly_EXPORTED_TARGETS})
