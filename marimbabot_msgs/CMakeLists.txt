cmake_minimum_required(VERSION 3.0.2)
project(marimbabot_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)