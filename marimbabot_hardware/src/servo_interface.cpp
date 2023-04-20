#include "marimbabot_hardware/servo_interface.hpp"

ServoInterface::ServoState::ServoState(const std::string &servo_name) {
    name = servo_name;
}

// Initializing the serial connection and servo_state object
ServoInterface::ServoInterface(ros::NodeHandle& node_handle, std::string &device, int baud) : 
    servo_state("mallet_finger") {
    
    ROS_INFO("Initializing servo interface");

    // Initializing the serial connection
    arduino_serial.setPort(device);
    arduino_serial.setBaudrate(baud);
    auto timeout = serial::Timeout::simpleTimeout(1000);
    arduino_serial.setTimeout(
        timeout.inter_byte_timeout,
        timeout.read_timeout_constant,
        timeout.read_timeout_multiplier,
        timeout.write_timeout_constant,
        timeout.write_timeout_multiplier
    );

    try_open_serial_port();
    
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
    servo_state.command = servo_state.position;
    previous_command = servo_state.command;
}

bool ServoInterface::try_open_serial_port() {
    try {
        arduino_serial.open();
        last_arduino_error = 0;
        ROS_INFO_STREAM("Servo controller: Successfully opened port " << arduino_serial.getPort());
        return true;
    } catch (serial::IOException &e) {
        if(e.getErrorNumber() != last_arduino_error) {
            ROS_ERROR_STREAM("Servo controller error: Unable to open port " << arduino_serial.getPort());
            ROS_ERROR_STREAM("Servo controller error: Error: " << e.what());
            last_arduino_error = e.getErrorNumber();
        }
        return false;
    }
}

void ServoInterface::read() {
    // Calculating time since last update
    ros::Time current_time = ros::Time::now();
    last_run_period = current_time - last_run_time;
    last_run_time = current_time;

    // Checking if the arduino is connected
    if(!arduino_serial.isOpen()) {
        // Trying to reconnect
        if(!try_open_serial_port()) {
            return;
        }
    }

    // Sending command to arduino
    try {
        arduino_serial.write("p\n");
        arduino_serial.flush();
    } catch (serial::SerialException &e) {
        ROS_ERROR_STREAM("Servo controller error: Unable to write to port " << arduino_serial.getPort());
        ROS_ERROR_STREAM("Servo controller error: Error: " << e.what());
        arduino_serial.close();
        return;
    }

    // Reading response from arduino
    std::string response;
    do {
        try {
            response += arduino_serial.readline();
        } catch (serial::SerialException &e) {
            ROS_ERROR_STREAM("Servo controller error: Unable to read from port " << arduino_serial.getPort());
            ROS_ERROR_STREAM("Servo controller error: Error: " << e.what());
            arduino_serial.close();
            return;
        }
        //ROS_INFO_STREAM("Servo controller: Received response from arduino: \"" << response << "\"");
    } while(arduino_serial.available() > 0);

    // Parsing response (extract int value)
    // remove first 2 chars and last chars (\r\n)
    response = response.substr(2, response.length() - 4);
    std::stringstream sstream(response);
    int value;
    sstream >> value;

    if(sstream.fail()) {
        ROS_WARN_STREAM("Servo controller error: Failed to parse position response from arduino");
        return;
    }

    // Remapping pwm value to radians
    servo_state.position = ((TOP_LIMIT - value) / ((double) TOP_LIMIT - BOTTOM_LIMIT)) * RADIAN_LIMIT;
}

void ServoInterface::write() {
    position_joint_saturation_interface.enforceLimits(last_run_period);

    // Checking if the arduino is connected
    // Not retrying to connect here as we already tried in read()
    if(!arduino_serial.isOpen()) {
        return;
    }

    // Not sending commands which already where before
    if(previous_command == servo_state.command) {
        return;
    }

    ROS_DEBUG_STREAM("Servo controller: Sending command to arduino: " << servo_state.command);

    previous_command = servo_state.command;

    // Remapping pwm value from radians input
    int command_value = TOP_LIMIT - (int) round((servo_state.command / RADIAN_LIMIT) * (TOP_LIMIT - BOTTOM_LIMIT));

    // Sending command to arduino
    std::stringstream sstream;
    sstream << "s " << command_value << "\n";

    try {
        arduino_serial.write(sstream.str());
        arduino_serial.flush();
    } catch (serial::SerialException &e) {
        ROS_ERROR_STREAM("Servo controller error: Failed to send command to arduino");
        ROS_ERROR_STREAM("Servo controller error: Error: " << e.what());
        arduino_serial.close();
        return;
    }

    // Reading response from arduino
    std::string response;
    do {
        try {
            response += arduino_serial.readline();
        } catch (serial::SerialException &e) {
            ROS_ERROR_STREAM("Servo controller error: Unable to read from port " << arduino_serial.getPort());
            ROS_ERROR_STREAM("Servo controller error: Error: " << e.what());
            arduino_serial.close();
            return;
        }
    } while(arduino_serial.available() > 0);

    // Removing trailing \r\n
    response = response.substr(0, response.length() - 2);

    // Checking if the command was successful
    if(response == "err_input_num") {
        ROS_ERROR_STREAM("Servo controller error: Number unparsable or below 0");
    }

    else if(response == "err_input_range") {
        ROS_ERROR_STREAM("Servo controller error: Number out of safety range");
    }

    else if(response == "err_cmd") {
        ROS_ERROR_STREAM("Servo controller error: Command not recognized");
    }

    else if(response == "ok") {
        // servo_state.position = servo_state.command;
    }

    else {
        ROS_ERROR_STREAM("Servo controller error: Unknown error");
    }
}