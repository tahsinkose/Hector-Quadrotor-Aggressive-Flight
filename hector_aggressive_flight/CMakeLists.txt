cmake_minimum_required(VERSION 2.8.3)
set (CMAKE_CXX_STANDARD 11)
project(hector_moveit_actions)

find_package(catkin REQUIRED COMPONENTS roscpp actionlib actionlib_msgs geometry_msgs hector_uav_msgs)


catkin_package(
    CATKIN_DEPENDS roscpp actionlib_msgs hector_uav_msgs
)


include_directories(${catkin_INCLUDE_DIRS})

add_executable(trajectory_controller src/aggressive_trajectory_executer.cpp)

target_link_libraries(trajectory_controller ${catkin_LIBRARIES})

add_dependencies(
  trajectory_controller
  ${hector_moveit_actions_EXPORTED_TARGETS} hector_uav_msgs_generate_messages
)