cmake_minimum_required(VERSION 3.0.2)
project(marimbabot_planning)

find_package(catkin REQUIRED COMPONENTS
  bio_ik
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  roscpp
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  trajectory_msgs
)

find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(SOURCES
    src/lilypond_to_hitpoint.cpp
    src/planning.cpp
)

add_executable(planning ${SOURCES})
target_link_libraries(planning ${catkin_LIBRARIES})

install(TARGETS planning
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
