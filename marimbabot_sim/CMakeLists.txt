cmake_minimum_required(VERSION 3.0.2)
project(marimbabot_sim)

find_package(catkin REQUIRED COMPONENTS
)

catkin_package(
)

## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
