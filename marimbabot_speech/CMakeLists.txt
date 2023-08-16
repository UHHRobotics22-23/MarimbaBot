cmake_minimum_required(VERSION 3.0.2)
project(marimbabot_speech)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    sound_play
    audio_common_msgs
    message_generation
)

catkin_python_setup()


generate_messages(
DEPENDENCIES
  std_msgs
)

catkin_package(
CATKIN_DEPENDS
  audio_common_msgs
  message_runtime
  std_msgs
  sound_play
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(
PROGRAMS
  src/keyowrd_spotting.py
  src/speech2text.py
  src/text2command.py
  src/text2speech.py
DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION}
)
