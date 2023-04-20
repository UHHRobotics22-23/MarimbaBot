#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include "marimbabot_hardware/servo_interface.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "hardware_control_node");

    ROS_INFO("Starting hardware control node");
    ros::NodeHandle node_handle;
    // TODO Parameterize the device
    std::string device = "/dev/ttyUSB0"; //"/dev/ttyUSB0";
    ServoInterface servo_interface(node_handle, device, 9600);
    controller_manager::ControllerManager controller_manager(&servo_interface);

    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate loop_rate(100);

    while(ros::ok()) {
        servo_interface.read();
        controller_manager.update(servo_interface.get_time(), servo_interface.get_period());
        servo_interface.write();
    
        loop_rate.sleep();
    }
}