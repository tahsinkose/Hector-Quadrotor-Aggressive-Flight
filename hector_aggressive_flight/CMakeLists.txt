cmake_minimum_required(VERSION 2.8.3)
set (CMAKE_CXX_STANDARD 11)
project(starmac)

find_package(catkin REQUIRED COMPONENTS roscpp actionlib actionlib_msgs geometry_msgs hector_uav_msgs)


catkin_package(
    INCLUDE_DIRS include
    LIBRARIES starmac_core
    CATKIN_DEPENDS roscpp actionlib_msgs hector_uav_msgs
)

include_directories(${catkin_INCLUDE_DIRS} include)

add_library(starmac_core
  src/starmac.cpp
)
target_link_libraries(starmac_core ${catkin_LIBRARIES})
add_dependencies(starmac_core ${catkin_EXPORTED_TARGETS})

add_executable(trajectory_controller src/aggressive_trajectory_executer.cpp)
target_link_libraries(trajectory_controller starmac_core ${catkin_LIBRARIES})
add_dependencies(
  trajectory_controller
  ${hector_moveit_actions_EXPORTED_TARGETS} hector_uav_msgs_generate_messages
)

install(TARGETS starmac_core trajectory_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
)

install(DIRECTORY cfg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
)