cmake_minimum_required(VERSION 3.0.2)
project(marimbabot_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  message_generation
  std_msgs
  audio_common_msgs
)

## Generate messages and actions
add_message_files(
  DIRECTORY msg
  FILES
  HitSequenceElement.msg
  CQTStamped.msg
  NoteOnset.msg
  Command.msg
  Speech.msg
  HitSequence.msg
  SequenceScore.msg
)

add_action_files(
  DIRECTORY action
  FILES
  HitSequence.action
  LilypondAudio.action
)

add_service_files(
  DIRECTORY srv
  FILES
  SpeechRecognition.srv
)

## Generate added messages and actions, services
generate_messages(
  DEPENDENCIES
  actionlib_msgs std_msgs  # Add any other message dependencies here
  audio_common_msgs
)

## Catkin package
catkin_package(
  CATKIN_DEPENDS message_runtime
)