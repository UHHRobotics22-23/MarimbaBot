cmake_minimum_required(VERSION 3.0.2)
project(marimbabot_audio)

find_package(catkin REQUIRED COMPONENTS
  audio_common_msgs
  cv_bridge
  message_generation
  sensor_msgs
  std_msgs
)

add_message_files(
DIRECTORY
  msg
FILES
  CQTStamped.msg
  NoteOnset.msg
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

catkin_install_python(
PROGRAMS
  scripts/detect_onset.py
  scripts/dump_audio.py
  scripts/note2midi.py
DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION}
)
