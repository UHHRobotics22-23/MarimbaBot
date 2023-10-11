/*
 * The implementation of the servo interface for the serial based servo controller.
 */
#include "marimbabot_hardware/servo_interface_serial.hpp"

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
    
    // The following registration to the interfaces connects the servo_state object to the 
    // controller manager and as such subsequently to the ros_controllers to enable the 
    // usage through ros_control

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

// Initializing the servo state object with current values
void ServoInterface::initialize() {
    // Keeping the current position of the servo
    read();
    servo_state.command = servo_state.position;
    previous_command = servo_state.command;
}

// Trying to open the serial port and load the servo limits on success
bool ServoInterface::try_open_serial_port() {
    try {
        arduino_serial.open();
        last_arduino_error = 0;
        ROS_INFO_STREAM("Servo controller: Successfully opened port " << arduino_serial.getPort());

        // Loading the servo limits from the arduino
        arduino_serial.write("l\n");

        // Reading response from arduino, no error handling here because we want to fail the connect
        std::string response;
        do {
            response += arduino_serial.readline();
        } while(arduino_serial.available() > 0);

        // Checking if response is long enough (l \r\n)
        if(response.length() < 4) {
            ROS_ERROR_STREAM("Servo controller error: Unable to read limits from arduino -> closing");
            arduino_serial.close();
            return false;
        }

        // Check if response starts with "l "
        if(response[0] != 'l' || response[1] != ' ') {
            ROS_ERROR_STREAM("Servo controller error: Limit response from arduino is invalid -> closing");
            arduino_serial.close();
            return false;
        }

        response = response.substr(2, response.length() - 4);
        
        try {
            // Splitting into top bottom and resolution
            // We could get into bounds errors with the substrings here, so we catch them
            std::vector<std::string> limits;
            std::string bottom_limit_str = response.substr(0, response.find(' '));
            response = response.substr(response.find(' ') + 1);
            std::string top_limit_str = response.substr(0, response.find(' '));
            response = response.substr(response.find(' ') + 1);
            std::string resolution_str = response;

            this->top_limit = atoi(top_limit_str.c_str());
            this->bottom_limit = atoi(bottom_limit_str.c_str());
            this->resolution = atoi(resolution_str.c_str());
            this->radian_limit = M_PI * ((this->top_limit - this->bottom_limit) / ((double) this->resolution));

            ROS_INFO_STREAM("Servo controller: Loaded limits from arduino");

            return true;
        } catch(std::exception &e) {
            ROS_ERROR_STREAM("Servo controller error: Unable to parse limits from arduino -> closing");
            arduino_serial.close();
            return false;
        }

    } catch (serial::IOException &e) {
        // Only print error if it is a new error
        if(e.getErrorNumber() != last_arduino_error) {
            ROS_ERROR_STREAM("Servo controller error: Unable to open port " << arduino_serial.getPort());
            ROS_ERROR_STREAM("Servo controller error: Error: " << e.what());
            last_arduino_error = e.getErrorNumber();
        }
        return false;
    }
}

void ServoInterface::read() {
    // Calculating time since last update this is needed for the controller manager
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

    // Sending get position (p) command to arduino
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
    } while(arduino_serial.available() > 0);

    
    // Checking if response is long enough (p \r\n)
    if(response.size() < 4) {
        ROS_WARN_STREAM("Servo controller error: Response from arduino is too short");
        return;
    }

    // Check if response starts with "p "
    if(response[0] != 'p' || response[1] != ' ') {
        ROS_WARN_STREAM("Servo controller error: Response from arduino does not start with 'p '");
        return;
    }
    
    // Parsing response (extract int value)
    // remove first 2 chars and last chars (\r\n
    response = response.substr(2, response.length() - 4);
    std::stringstream sstream(response);
    int value;
    sstream >> value;

    if(sstream.fail()) {
        ROS_WARN_STREAM("Servo controller error: Failed to parse position response from arduino");
        return;
    }

    // Remapping pwm value to radians
    servo_state.position = ((top_limit - value) / ((double) top_limit - bottom_limit)) * radian_limit;
}

void ServoInterface::write() {
    // Enforcing limits on the servo command as specified in the joint_limits.yaml
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

    // Remapping pwm value from radians input
    int command_value = top_limit - (int) round((servo_state.command / radian_limit) * (top_limit - bottom_limit));

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
        ROS_ERROR_STREAM_THROTTLE(10, "Servo controller error: Number unparsable or below 0");
    }

    else if(response == "err_input_range") {
        ROS_ERROR_STREAM("Servo controller error: Number out of safety range");
    }

    else if(response == "err_cmd") {
        ROS_ERROR_STREAM("Servo controller error: Command not recognized");
    }

    else if(response == "ok") {
        previous_command = servo_state.command;
    }

    else {
        ROS_ERROR_STREAM("Servo controller error: Unknown error");
    }
}