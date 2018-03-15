cmake_minimum_required(VERSION 2.8.3)
project(sarafun_tree)

find_package(catkin REQUIRED COMPONENTS
  behavior_tree_leaves
  behavior_tree_core
  sarafun_manipulation
  sarafun_assembly
  roscpp
  roslib
  std_msgs
  std_srvs
  actionlib_msgs
  actionlib
  message_generation
  sarafun_msgs
)

add_service_files(
  DIRECTORY srv
  FILES LoadTree.srv
)

link_directories(${catkin_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
set(CMAKE_BUILD_TYPE Debug)
add_definitions(-g -O0)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs behavior_tree_leaves behavior_tree_core message_runtime
  INCLUDE_DIRS include
  LIBRARIES tree_runner
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(tree_runner src/TreeRunner.cpp src/parse_tree.cpp)
target_link_libraries(tree_runner ${catkin_LIBRARIES})

add_executable(sarafun_bt_demo src/sarafun_bt_demo.cpp)
target_link_libraries(sarafun_bt_demo tree_runner ${catkin_LIBRARIES})

add_executable(sarafun_bt src/sarafun_bt.cpp)
target_link_libraries(sarafun_bt tree_runner ${catkin_LIBRARIES})
add_dependencies(sarafun_bt ${catkin_EXPORTED_TARGETS} sarafun_tree_generate_messages_cpp)

## Demo nodes

add_executable(GrabObjectAction src/demo_bt_nodes/GrabObjectAction.cpp)
target_link_libraries(GrabObjectAction ${catkin_LIBRARIES})

add_executable(ApproachObjectsAction src/demo_bt_nodes/ApproachObjectsAction.cpp)
target_link_libraries(ApproachObjectsAction ${catkin_LIBRARIES})

add_executable(FoldingAssemblyAction src/demo_bt_nodes/FoldingAssemblyAction.cpp)
target_link_libraries(FoldingAssemblyAction ${catkin_LIBRARIES})

add_executable(InsertionWithDeformationAction src/demo_bt_nodes/InsertionWithDeformationAction.cpp)
target_link_libraries(InsertionWithDeformationAction ${catkin_LIBRARIES})

add_executable(PlaceActionDemo src/demo_bt_nodes/PlaceAction.cpp)
target_link_libraries(PlaceActionDemo ${catkin_LIBRARIES})

add_executable(IsSimple src/demo_bt_nodes/IsSimple.cpp)
target_link_libraries(IsSimple ${catkin_LIBRARIES})

## Sarafun tree client nodes

add_executable(AlignAction src/sarafun_action_nodes/AlignAction.cpp)
target_link_libraries(AlignAction ${catkin_LIBRARIES})

add_executable(AssembledAction src/sarafun_action_nodes/AssembledAction.cpp)
target_link_libraries(AssembledAction ${catkin_LIBRARIES})

add_executable(ContactAction src/sarafun_action_nodes/ContactAction.cpp)
target_link_libraries(ContactAction ${catkin_LIBRARIES})

add_executable(GraspAction src/sarafun_action_nodes/GraspAction.cpp)
target_link_libraries(GraspAction ${catkin_LIBRARIES})

add_executable(InitialAction src/sarafun_action_nodes/InitialAction.cpp)
target_link_libraries(InitialAction ${catkin_LIBRARIES})

add_executable(MoveAction src/sarafun_action_nodes/MoveAction.cpp)
target_link_libraries(MoveAction ${catkin_LIBRARIES})

add_executable(PickupAction src/sarafun_action_nodes/PickupAction.cpp)
target_link_libraries(PickupAction ${catkin_LIBRARIES})

add_executable(RetractAction src/sarafun_action_nodes/RetractAction.cpp)
target_link_libraries(RetractAction ${catkin_LIBRARIES})

add_executable(InsertAction src/sarafun_action_nodes/InsertAction.cpp)
target_link_libraries(InsertAction ${catkin_LIBRARIES})

add_executable(DeformAction src/sarafun_action_nodes/DeformAction.cpp)
target_link_libraries(DeformAction ${catkin_LIBRARIES})
