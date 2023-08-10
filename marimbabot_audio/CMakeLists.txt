cmake_minimum_required(VERSION 3.0.2)
project(marimbabot_audio)

find_package(catkin REQUIRED COMPONENTS
  audio_common_msgs
  cv_bridge
  message_generation
  rospy
  sensor_msgs
  sound_play
  std_msgs
)

generate_messages(
DEPENDENCIES
  std_msgs
)

catkin_package(
CATKIN_DEPENDS
  audio_common_msgs
  message_runtime
  sensor_msgs
  std_msgs
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(
PROGRAMS
  src/audio_from_lilypond.py
  scripts/onset_detection.py
  scripts/onsets_visualization.py
DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION}
)