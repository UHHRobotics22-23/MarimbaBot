cmake_minimum_required(VERSION 3.0.2)
project(marimbabot_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  message_generation
  std_msgs
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
  TmpFile.msg
)

add_action_files(
  DIRECTORY action
  FILES
  HitSequence.action
)

## Generate added messages and actions
generate_messages(
  DEPENDENCIES
  actionlib_msgs std_msgs  # Add any other message dependencies here
)

## Catkin package
catkin_package(
  CATKIN_DEPENDS message_runtime
)