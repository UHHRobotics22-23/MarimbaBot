#include "marimbabot_hardware/servo_interface.hpp"

ServoInterface::ServoState::ServoState(const std::string &servo_name) {
    name = servo_name;
}

// Initializing the serial connection and servo_state object
ServoInterface::ServoInterface(ros::NodeHandle& node_handle, std::string &device, int baud) : 
    arduino_serial(device, baud, serial::Timeout::simpleTimeout(1000)),
    servo_state("mallet_servo") {
    
    ROS_INFO("Initializing servo interface");
    
    // Creating the joint state interface + handle
    hardware_interface::JointStateHandle servo_state_handle(
        servo_state.name, &servo_state.position, &servo_state.velocity, &servo_state.effort);
    joint_state_interface.registerHandle(servo_state_handle);

    registerInterface(&joint_state_interface);

    // Creating the servo position control interface + handle
    hardware_interface::JointHandle position_servo_handle(
        joint_state_interface.getHandle(servo_state.name), &servo_state.command);
    position_joint_interface.registerHandle(position_servo_handle);

    registerInterface(&position_joint_interface);

    // Creating the joint saturation interface + handle
    joint_limits_interface::getJointLimits(servo_state.name, node_handle, servo_limits);
    joint_limits_interface::PositionJointSaturationHandle saturation_handle(
        position_servo_handle, servo_limits);
    position_joint_saturation_interface.registerHandle(saturation_handle);

    registerInterface(&position_joint_saturation_interface);
}

void ServoInterface::read() {
    // Calculating time since last update
    ros::Time current_time = ros::Time::now();
    last_run_period = current_time - last_run_time;
    last_run_time = current_time;

    // This is a stub as we have no feedback from the controller
    // Could add a signal after finished movement of arduino

    // TODO Check if servo controller is ready to receive new commands (potentially needs arduino code change)
}

void ServoInterface::write() {
    position_joint_saturation_interface.enforceLimits(last_run_period);

    // Checking if the arduino is connected
    if(!arduino_serial.isOpen()) {
        ROS_WARN_STREAM("Arduino is not connected");
        return;
    }

    // Not sending commands which already where before
    if(previous_command == servo_state.command) {
        // ROS_INFO_STREAM("Command already sent");
        // ROS_INFO_STREAM("Previous command: " << previous_command);
        return;
    }

    previous_command = servo_state.command;

    // Remapping pwm value from radians input
    int command_value = (int) round((servo_state.command / (2 * M_PI)) * 255);

    std::stringstream sstream;
    sstream << "servoPos " << command_value << "\n";
    // ROS_INFO_STREAM(sstream.str());
    arduino_serial.write(sstream.str());
    std::string response;
    while(arduino_serial.available() > 0) {
        response = arduino_serial.readline();

        // ROS_INFO_STREAM(response.c_str());
    }
    

    // TODO Parse errors from servo controller
    if(response == "") {
        // ROS_WARN_STREAM("Could not set servo state");
    }

    if(response == "ok") {
        // Stub
    }

    servo_state.position = servo_state.command;
}