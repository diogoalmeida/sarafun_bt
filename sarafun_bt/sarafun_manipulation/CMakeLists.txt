cmake_minimum_required(VERSION 2.8.3)
project(sarafun_manipulation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sarafun_generic_al_server
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

add_executable(ApproachDummy src/ApproachDummy.cpp)
target_link_libraries(ApproachDummy ${catkin_LIBRARIES})
add_dependencies(ApproachDummy ${sarafun_manipulation_EXPORTED_TARGETS})

add_executable(GrabDummy src/GrabDummy.cpp)
target_link_libraries(GrabDummy ${catkin_LIBRARIES})
add_dependencies(GrabDummy ${sarafun_manipulation_EXPORTED_TARGETS})

add_executable(PlaceDummy src/PlaceDummy.cpp)
target_link_libraries(PlaceDummy ${catkin_LIBRARIES})
add_dependencies(PlaceDummy ${sarafun_manipulation_EXPORTED_TARGETS})
