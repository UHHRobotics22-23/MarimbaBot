cmake_minimum_required(VERSION 3.0.2)
project(marimbabot_planning)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
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
  CATKIN_DEPENDS actionlib_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(planning src/planning.cpp)
add_executable(lilypond_to_hitpoint src/lilypond_to_hitpoint.cpp)
target_link_libraries(planning ${catkin_LIBRARIES})
target_link_libraries(lilypond_to_hitpoint ${catkin_LIBRARIES})

install(TARGETS planning lilypond_to_hitpoint
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Action 
add_action_files(
  DIRECTORY $(find marimbabot_description)/actions
  FILES HitSequence.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs  # Or other packages containing msgs
)