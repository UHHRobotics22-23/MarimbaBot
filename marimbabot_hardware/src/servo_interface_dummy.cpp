#include "marimbabot_hardware/servo_interface_dummy.hpp"

ServoInterface::ServoState::ServoState(const std::string &servo_name) {
    name = servo_name;
}

// Initializing the serial connection and servo_state object
ServoInterface::ServoInterface(ros::NodeHandle& node_handle) : 
    servo_state("mallet_finger") {
    
    ROS_INFO("Initializing dummy servo interface");
    
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

void ServoInterface::initialize() {
    // Keeping the current position of the servo
    read();
    // For the dummy the starting position is 0
    servo_state.position = 0;
    servo_state.command = servo_state.position;
    previous_command = servo_state.command;

    // Setting the limits similar like in the hardware
    this->top_limit = 140;
    this->bottom_limit = 70;
    this->resolution = 180;
    this->radian_limit = M_PI * ((this->top_limit - this->bottom_limit) / ((double) this->resolution));
}

void ServoInterface::read() {
    // Calculating time since last update
    ros::Time current_time = ros::Time::now();
    last_run_period = current_time - last_run_time;
    last_run_time = current_time;

    // Nothing to read from dummy servo
}

void ServoInterface::write() {
    position_joint_saturation_interface.enforceLimits(last_run_period);

    // Not sending commands which already where before
    if(previous_command == servo_state.command) {
        return;
    }

    ROS_DEBUG_STREAM("Servo dummy controller: Sending command to arduino: " << servo_state.command);

    // Doing this in the dummy to include the effect of the rounding

    // Remapping pwm value from radians input
    int command_value = top_limit - (int) round((servo_state.command / radian_limit) * (top_limit - bottom_limit));

    // Setting inbetween the conversions
    previous_command = servo_state.command;

    // Remapping pwm value to radians
    servo_state.position = ((top_limit - command_value) / ((double) top_limit - bottom_limit)) * radian_limit;
}