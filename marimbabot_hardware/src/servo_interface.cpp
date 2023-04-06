#include "marimbabot_hardware/servo_interface.hpp"

ServoInterface::ServoState::ServoState(const std::string &servo_name) {
    name = servo_name;
}

ServoInterface::ServoInterface(const std::string &device, int baud) : 
    arduino_serial(device, baud, serial::Timeout::simpleTimeout(1000)),
    servo_state("mallet_servo") {

    hardware_interface::JointStateHandle servo_state_handle(
        servo_state.name, &servo_state.position, &servo_state.velocity, &servo_state.effort);
    joint_state_interface.registerHandle(servo_state_handle);

    registerInterface(&joint_state_interface);

    hardware_interface::JointHandle position_servo_handle(
        joint_state_interface.getHandle(servo_state.name), &servo_state.command);
    position_joint_interface.registerHandle(position_servo_handle);

    registerInterface(&position_joint_interface);
}

void ServoInterface::read() {
    ros::Time current_time = ros::Time::now();
    last_run_period = current_time - last_run_time;
    last_run_time = current_time;

    // This is a stub as we have no feedback from the controller
    // Could add a signal after finished movement of arduino
}

void ServoInterface::write() {
    if(!arduino_serial.isOpen()) {
        return;
    }

    if(previous_command == servo_state.command) {
        return;
    }

    previous_command = servo_state.command;

    int command_value = (int) round((servo_state.command / 2 * M_PI) * 255);

    std::stringstream sstream;
    sstream << "servoPos " << command_value << "\n";
    arduino_serial.write(sstream.str());
    std::string response = arduino_serial.readline();

    if(response == "") {
        ROS_WARN_STREAM("Could not set servo state");
    }

    if(response == "ok") {
        servo_state.position = servo_state.command;
    }
}