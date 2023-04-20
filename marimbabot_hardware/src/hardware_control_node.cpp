#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include "marimbabot_hardware/servo_interface.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "hardware_control_node");

    // Get device parameters from parameter server
    ros::NodeHandle node_handle;
    int baud;
    std::string device;
    node_handle.param("mallet_finger_device", device, std::string("/dev/ttyUSB0"));
    node_handle.param("mallet_finger_baud", baud, 115200);

    ROS_INFO("Starting hardware control node for device %s", device.c_str());

    ServoInterface servo_interface(node_handle, device, baud);
    controller_manager::ControllerManager controller_manager(&servo_interface);

    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate loop_rate(100);

    servo_interface.initialize();

    // Controller loop
    while(ros::ok()) {
        servo_interface.read();
        controller_manager.update(servo_interface.get_time(), servo_interface.get_period());
        servo_interface.write();
    
        loop_rate.sleep();
    }
}