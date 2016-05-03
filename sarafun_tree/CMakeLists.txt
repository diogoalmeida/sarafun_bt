cmake_minimum_required(VERSION 2.8.3)
project(sarafun_tree)

find_package(catkin REQUIRED COMPONENTS
  behavior_tree_leaves
  behavior_tree_core
  roscpp
  std_msgs
  actionlib_msgs
  actionlib
)

link_directories(${catkin_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

catkin_package(
  CATKIN_DEPENDS actionlib_msgs behavior_tree_leaves behavior_tree_core
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(GrabObjectAction src/GrabObjectAction.cpp)
target_link_libraries(GrabObjectAction ${catkin_LIBRARIES})

add_executable(ApproachObjectsAction src/ApproachObjectsAction.cpp)
target_link_libraries(ApproachObjectsAction ${catkin_LIBRARIES})

add_executable(FoldingAssemblyAction src/FoldingAssemblyAction.cpp)
target_link_libraries(FoldingAssemblyAction ${catkin_LIBRARIES})

add_executable(InsertionWithDeformationAction src/InsertionWithDeformationAction.cpp)
target_link_libraries(InsertionWithDeformationAction ${catkin_LIBRARIES})

add_executable(PlaceAction src/PlaceAction.cpp)
target_link_libraries(PlaceAction ${catkin_LIBRARIES})

add_executable(sarafun_bt_demo src/sarafun_bt_demo.cpp src/parse_tree.cpp)
target_link_libraries(sarafun_bt_demo ${catkin_LIBRARIES})
