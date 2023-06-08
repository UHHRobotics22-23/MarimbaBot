cmake_minimum_required(VERSION 3.0.2)
project(marimbabot_command_recognition)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    std_msgs
    audio_common_msgs
    message_generation
    sound_play
)

catkin_python_setup()

add_message_files(
DIRECTORY
  msg
    FILES
        Command.msg
        Speech.msg
        TmpFile.msg
        Speaking.msg
        SpeakingEnd.msg
)

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
  src/audio2tmp.py
  src/speech2text.py
  src/text2command.py
  src/text2speech.py
DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION}
)
