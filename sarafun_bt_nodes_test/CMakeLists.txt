cmake_minimum_required(VERSION 2.8.3)
project(sarafun_bt_nodes_test)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sarafun_generic_al_server
  sarafun_msgs
  actionlib_msgs
  actionlib
)

link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
  CATKIN_DEPENDS actionlib_msgs sarafun_msgs sarafun_generic_al_server
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(AlignDummy src/AlignDummy.cpp)
target_link_libraries(AlignDummy ${catkin_LIBRARIES})
add_dependencies(AlignDummy ${catkin_EXPORTED_TARGETS})

add_executable(AssembledDummy src/AssembledDummy.cpp)
target_link_libraries(AssembledDummy ${catkin_LIBRARIES})
add_dependencies(AssembledDummy ${catkin_EXPORTED_TARGETS})

add_executable(ContactDummy src/ContactDummy.cpp)
target_link_libraries(ContactDummy ${catkin_LIBRARIES})
add_dependencies(ContactDummy ${catkin_EXPORTED_TARGETS})

add_executable(GraspDummy src/GraspDummy.cpp)
target_link_libraries(GraspDummy ${catkin_LIBRARIES})
add_dependencies(GraspDummy ${catkin_EXPORTED_TARGETS})

add_executable(InitialDummy src/InitialDummy.cpp)
target_link_libraries(InitialDummy ${catkin_LIBRARIES})
add_dependencies(InitialDummy ${catkin_EXPORTED_TARGETS})

add_executable(MoveDummy src/MoveDummy.cpp)
target_link_libraries(MoveDummy ${catkin_LIBRARIES})
add_dependencies(MoveDummy ${catkin_EXPORTED_TARGETS})

add_executable(PickupDummy src/PickupDummy.cpp)
target_link_libraries(PickupDummy ${catkin_LIBRARIES})
add_dependencies(PickupDummy ${catkin_EXPORTED_TARGETS})

add_executable(RetractDummy src/RetractDummy.cpp)
target_link_libraries(RetractDummy ${catkin_LIBRARIES})
add_dependencies(RetractDummy ${catkin_EXPORTED_TARGETS})
