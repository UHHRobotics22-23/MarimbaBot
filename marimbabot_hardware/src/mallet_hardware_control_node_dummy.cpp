/*
 * This node implements the main loop for the dummy servo hardware interface.
 */
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include "marimbabot_hardware/servo_interface_dummy.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "mallet_hardware_control_node");

    // Get device parameters from parameter server
    ros::NodeHandle node_handle { "~" };
    int top_limit;
    int bottom_limit;

    // Initialize the servo interface and controller manager
    ServoInterface servo_interface(node_handle);
    controller_manager::ControllerManager controller_manager(&servo_interface);

    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate loop_rate(100);

    servo_interface.initialize();

    // Controller loop
    // This is the main functionality of the node.
    // It reads the state of the dummy servo motor, updates the controller manager, 
    // and writes the command to the dummy servo motor.
    while(ros::ok()) {
        servo_interface.read();
        controller_manager.update(servo_interface.get_time(), servo_interface.get_period());
        servo_interface.write();
    
        loop_rate.sleep();
    }
}