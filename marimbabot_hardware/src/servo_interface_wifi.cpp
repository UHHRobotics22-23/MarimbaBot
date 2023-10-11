#include "marimbabot_hardware/servo_interface_wifi.hpp"

ServoInterface::ServoState::ServoState(const std::string &servo_name) { name = servo_name; }

// Initializing the serial connection and servo_state object
ServoInterface::ServoInterface(ros::NodeHandle &node_handle, std::string &address, int port)
    : servo_state("mallet_finger") {
    ROS_INFO("Initializing servo interface");
    // Convert Adress String into Char
    const int length = address.length();
    char *char_address = new char[length + 1];
    strcpy(char_address, address.c_str());

    // Creating socket file descriptor
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        ROS_ERROR_STREAM("No Socket was created");
        return;
    }
    // creating space for adresses
    memset(&servaddr, 0, sizeof(servaddr));
    memset(&cliaddr, 0, sizeof(cliaddr));

    // Filling server information
    servaddr.sin_family = AF_INET;                           // IPv4
    inet_pton(AF_INET, char_address, &(servaddr.sin_addr));  // IPv4
    servaddr.sin_port = htons(PORT);
    //Call function to open UDP Port 
    try_open_udp_port();

    // Creating the joint state interface + handle
    hardware_interface::JointStateHandle servo_state_handle(servo_state.name, &servo_state.position,
                                                            &servo_state.velocity, &servo_state.effort);
    joint_state_interface.registerHandle(servo_state_handle);

    registerInterface(&joint_state_interface);

    // Creating the servo position control interface + handle
    hardware_interface::JointHandle position_servo_handle(joint_state_interface.getHandle(servo_state.name),
                                                          &servo_state.command);
    position_joint_interface.registerHandle(position_servo_handle);

    registerInterface(&position_joint_interface);

    // Creating the joint saturation interface + handle
    joint_limits_interface::getJointLimits(servo_state.name, node_handle, servo_limits);
    joint_limits_interface::PositionJointSaturationHandle saturation_handle(position_servo_handle, servo_limits);
    position_joint_saturation_interface.registerHandle(saturation_handle);

    registerInterface(&position_joint_saturation_interface);
}

void ServoInterface::initialize() {
    // Keeping the current position of the servo
    read();
    servo_state.command = servo_state.position;
    previous_command = servo_state.command;
}

void ServoInterface::try_open_udp_port() {
    // Bind the Socket with the address information
    bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr));

    ROS_INFO("Servo controller: Successfully opened port ");

    len = sizeof(cliaddr);  // len is value/result
    //calls send_and_receive functin with the limit_Sender. ROS recevies the limits from the hardware.
    send_and_receive(ServoInterface::l_Sender);
}
void ServoInterface::read() {
    // Calculating time since last update
    ros::Time current_time = ros::Time::now();
    last_run_period = current_time - last_run_time;
    last_run_time = current_time;
    //If ROS still needs the limits of the hardware, the limits are queried in advance.
    while (limit_checker == 0) {
        send_and_receive(ServoInterface::l_Sender);
    }
    //The current position of the hardware is queried.
    send_and_receive(ServoInterface::p_Sender);
    return;
}
void ServoInterface::write() {
     // Enforcing limits on the servo command as specified in the joint_limits.yaml
    position_joint_saturation_interface.enforceLimits(last_run_period);
    // Not sending commands which already where before
    if (previous_command == servo_state.command) {
        return;
    }
    ROS_DEBUG_STREAM("Servo controller: Sending command to arduino: " << servo_state.command);

    // Remapping pwm value from radians input
    int command_value = top_limit - (int)round((servo_state.command / radian_limit) * (top_limit - bottom_limit));
    // Sending command to arduino via the send_and_receive function
    std::string s_Sender = "s ";

    s_Sender += std::to_string(command_value);
    s_Sender += ("\n");
    char *write_pos_array = convert_to_char(s_Sender);

    send_and_receive(write_pos_array);
    delete[] write_pos_array;
}

void ServoInterface::send_and_receive(const char *message) {
    //create space for buffer
    memset(&buffer, 0, sizeof(buffer));
    //send the message to the arduino
    sendto(sockfd, (const char *)message, strlen(message), MSG_CONFIRM, (const struct sockaddr *)&servaddr, len);
    //receive the message from arduino and store it in the buffer
    received_message = recvfrom(sockfd, (char *)buffer, MAXLINE, MSG_DONTWAIT, (struct sockaddr *)&servaddr, &len);
    buffer[received_message] = '\0';
    std::string response;
    // int size_arr = sizeof(buffer) / sizeof(char);
    response = buffer;
    // check the content of the received response from the arduino and call the related function
    if (response[0] == 'l') {
        receive_limits_function(response);
    }

    else if (response[0] == 'p') {
        receive_postion_function(response);
    }

    else if (response == "err_input_num") {
        ROS_ERROR_STREAM_THROTTLE(10, "Servo controller error: Number unparsable or below 0");
    }

    else if (response == "err_input_range") {
        ROS_ERROR_STREAM("Servo controller error: Number out of safety range");
    }

    else if (response == "err_cmd") {
        ROS_ERROR_STREAM("Servo controller error: Command not recognized");
    }

    else if (response == "ok") {
        previous_command = servo_state.command;
       
    }

    else if (response == "") {
        // We didn't receive anything -> fine
    }

    else {
     
        ROS_ERROR_STREAM("Servo controller error: Unknown error");
    }
}
void ServoInterface::receive_limits_function(std::string response) {
    //check length 
    if (response.length() < 4) {
        ROS_ERROR_STREAM("Servo controller error: Unable to read limits from arduino -> closing");
        close(sockfd);
        return;
    }
    //check, if the limits are already loaded
    if (limit_checker == 1){
        //limits already loaded
        return;
    }

    response = response.substr(2, response.length() - 2);

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
        this->radian_limit = (M_PI) * ((this->top_limit - this->bottom_limit) / ((double)this->resolution));

        ROS_INFO_STREAM("Servo controller: Loaded limits from arduino");

        limit_checker = 1;
        return;
    } catch (std::exception &e) {
        ROS_ERROR_STREAM("Servo controller error: Unable to parse limits from arduino -> closing");
        close(sockfd);
        return;
    }
}
void ServoInterface::receive_postion_function(std::string response) {
    //check length 
    if (response.size() < 4) {
        ROS_WARN_STREAM("Servo controller error: Response from arduino is too short");
        return;
    }
    //update the position
    response = response.substr(2, response.length());
    std::stringstream sstream(response);
    int value;
    sstream >> value;
    if (sstream.fail()) {
        ROS_WARN_STREAM("Servo controller error: Failed to parse position response from arduino");
        return;
    }

    // Remapping pwm value to radians
    servo_state.position = ((top_limit - value) / ((double)top_limit - bottom_limit)) * radian_limit;
}
//simple function to convert a sting into char
char *ServoInterface::convert_to_char(std::string str) {
    int length = str.length();

    // declaring character array (+1 for null terminator)
    char *char_array = new char[length + 1];
    // copying the contents of the
    // string to char array
    strcpy(char_array, str.c_str());
    return char_array;
}