cmake_minimum_required(VERSION 3.0.2)
project(marimbabot_planning)

find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning_interface
  roscpp
  tf2_eigen
  tf2_ros
  geometry_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(move_to_pose src/move_to_pose.cpp)
target_link_libraries(move_to_pose ${catkin_LIBRARIES}) 

install(TARGETS move_to_pose
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
