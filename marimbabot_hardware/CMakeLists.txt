cmake_minimum_required(VERSION 3.0.2)
project(marimbabot_hardware)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  controller_manager
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(mallet_hardware_control_node_wifi src/mallet_hardware_control_node_wifi.cpp src/servo_interface_wifi.cpp)

target_link_libraries(mallet_hardware_control_node_wifi
  ${catkin_LIBRARIES}
)