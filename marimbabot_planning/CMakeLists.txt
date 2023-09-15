cmake_minimum_required(VERSION 3.0.2)
project(marimbabot_planning)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  bio_ik
  geometry_msgs
  marimbabot_msgs
  moveit_msgs
  moveit_ros_planning_interface
  interactive_markers
  roscpp
  std_msgs
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  trajectory_msgs
  backward_ros
)

# Include Python3 for the matplotlib-cpp library
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

# Include Eigen3
find_package(Eigen3 REQUIRED)

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
    src/double_mallet.cpp
)

add_executable(planning ${SOURCES})
target_link_libraries(planning ${catkin_LIBRARIES} Python3::Python Python3::Module)

add_executable(double ${SOURCES})
target_link_libraries(double ${catkin_LIBRARIES} Python3::Python Python3::Module)

install(TARGETS planning
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS double
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
