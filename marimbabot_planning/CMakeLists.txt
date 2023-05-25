cmake_minimum_required(VERSION 3.0.2)
project(marimbabot_planning)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  bio_ik
  geometry_msgs
  marimbabot_msgs
  moveit_msgs
  moveit_ros_planning_interface
  roscpp
  std_msgs
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  trajectory_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(SOURCES
    src/planning.cpp
    src/utils.cpp
)

add_executable(planning ${SOURCES})
target_link_libraries(planning ${catkin_LIBRARIES})

install(TARGETS planning
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
