#include "marimbabot_hardware/servo_interface.hpp"

ServoInterface::ServoState::ServoState(const std::string& servo_name) {
    name = servo_name;
}

// Initializing the UDP connection and servo_state object
ServoInterface::ServoInterface(ros::NodeHandle& node_handle, std::string& host, int port) :
    servo_state("mallet_finger") {

    ROS_INFO("Initializing servo interface");

    // Initializing the UDP connection
    udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_socket < 0) {
        ROS_ERROR("Servo controller error: Failed to create UDP socket");
        return;
    }

    // Setting up server address
    memset((char*)&server_address, 0, sizeof(server_address));
    server_address.sin_family = AF_INET;
    server_address.sin_port = htons(port);
    if (inet_aton(host.c_str(), &server_address.sin_addr) == 0) {
        ROS_ERROR("Servo controller error: Invalid server address");
        return;
    }

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

bool ServoInterface::try_open_udp_socket() {
    // UDP socket is already created in the constructor, so no need to open it here
    return true;
}

void ServoInterface::read() {
    // Calculating time since last update
    ros::Time current_time = ros::Time::now();
    last_run_period = current_time - last_run_time;
    last_run_time = current_time;

    // Checking if the UDP socket is open
    if (udp_socket < 0) {
        // Trying to reconnect
        if (!try_open_udp_socket()) {
            return;
        }
    }

    // Sending command to server
    std::string command = "p\n";
    if (sendto(udp_socket, command.c_str(), command.length(), 0, (struct sockaddr*)&server_address,
               sizeof(server_address)) < 0) {
        ROS_ERROR("Servo controller error: Failed to send command to server");
        return;
    }

    // Reading response from server
    char buffer[256];
    memset(buffer, 0, sizeof(buffer));
    socklen_t server_address_length = sizeof(server_address);
    int num_bytes = recvfrom(udp_socket, buffer, sizeof(buffer) - 1, 0,
                             (struct sockaddr*)&server_address, &server_address_length);
    if (num_bytes < 0) {
        ROS_ERROR("Servo controller error: Failed to receive response from server");
        return;
    }

    // Parsing the received response
    buffer[num_bytes] = '\0';
    std::string response(buffer);
    try {
        servo_state.position = std::stod(response);
    } catch (const std::exception& e) {
        ROS_ERROR("Servo controller error: Failed to parse response");
    }
}

void ServoInterface::write() {
    // Calculating time since last update
    ros::Time current_time = ros::Time::now();
    last_run_period = current_time - last_run_time;
    last_run_time = current_time;

    // Checking if the UDP socket is open
    if (udp_socket < 0) {
        // Trying to reconnect
        if (!try_open_udp_socket()) {
            return;
        }
    }

    // Checking if the command has changed
    if (servo_state.command != previous_command) {
        // Sending command to server
        std::string command = "s " + std::to_string(servo_state.command) + "\n";
        if (sendto(udp_socket, command.c_str(), command.length(), 0,
                   (struct sockaddr*)&server_address, sizeof(server_address)) < 0) {
            ROS_ERROR("Servo controller error: Failed to send command to server");
            return;
        }

        // Reading response from server
        char buffer[256];
        memset(buffer, 0, sizeof(buffer));
        socklen_t server_address_length = sizeof(server_address);
        int num_bytes = recvfrom(udp_socket, buffer, sizeof(buffer) - 1, 0,
                                 (struct sockaddr*)&server_address, &server_address_length);
        if (num_bytes < 0) {
            ROS_ERROR("Servo controller error: Failed to receive response from server");
            return;
        }

        // Parsing the received response
        buffer[num_bytes] = '\0';
        std::string response(buffer);
        if (response != "OK") {
            ROS_ERROR("Servo controller error: Command execution failed");
        }

        // Updating the previous command
        previous_command = servo_state.command;
    }
}
