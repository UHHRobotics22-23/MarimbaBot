#include "marimbabot_hardware/servo_interface_wifi.hpp"

ServoInterface::ServoState::ServoState(const std::string &servo_name) {
    name = servo_name;
}

// Initializing the serial connection and servo_state object
ServoInterface::ServoInterface(ros::NodeHandle& node_handle, std::string &address, int port) : 
    servo_state("mallet_finger") {
    
    ROS_INFO("Initializing servo interface");
    //Convert Adress String into Char
    const int length = address.length();
    char* char_address = new char[length + 1];
    strcpy(char_address, address.c_str());
    
       
    // Creating socket file descriptor
    if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
        ROS_ERROR_STREAM("No Socket was created");
        return;
    }
    //creating space for adresses
    memset(&servaddr, 0, sizeof(servaddr));
    memset(&cliaddr, 0, sizeof(cliaddr));

    
    // Filling server information
    servaddr.sin_family = AF_INET; // IPv4
    inet_pton(AF_INET, char_address, &(servaddr.sin_addr)); // IPv4
    servaddr.sin_port = htons(PORT);

                    // arduino_serial.setTimeout(
                    //     timeout.inter_byte_timeout,
                    //     timeout.read_timeout_constant,
                    //     timeout.read_timeout_multiplier,
                    //     timeout.write_timeout_constant,
                    //     timeout.write_timeout_multiplier
                    // );

     try_open_udp_port();
    
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

bool ServoInterface::try_open_udp_port() {
    try {
        //Bind the Socket with the address information
        bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)); 

        ROS_INFO("Servo controller: Successfully opened port " );

        // Loading the servo limits from the arduino

        //send "l " to get the information from the Mallet
        len = sizeof(cliaddr);  //len is value/result
        sendto(sockfd, (const char *)l_Sender, strlen(l_Sender),MSG_CONFIRM, (const struct sockaddr *) &servaddr,len);
        ROS_INFO("Message to get limits was sent");

        received_message = recvfrom(sockfd, (char *)buffer, MAXLINE, 
                MSG_WAITALL, ( struct sockaddr *) &servaddr,
                &len);
         buffer[received_message] = '\0';
        ROS_INFO("Message Received");
        ROS_INFO("The Servo interface received the following message from the mallet holder: %s",buffer);
        std::string response; 
	    response = buffer;
        // Reading response from arduino, no error handling here because we want to fail the connect
        
        //Checking if response is long enough (l \r\n)
        if(response.length() < 4) {
            ROS_ERROR_STREAM("Servo controller error: Unable to read limits from arduino -> closing");
            close(sockfd);
            return false;
        }

        // // Check if response starts with "l "
        if(response[0] != 'l' || response[1] != ' ') {
            ROS_ERROR_STREAM("Servo controller error: Limit response from arduino is invalid -> closing");
            close(sockfd);
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
            this->radian_limit = (2 * M_PI) * ((this->top_limit - this->bottom_limit) / ((double) this->resolution));

            ROS_INFO_STREAM("Servo controller: Loaded limits from arduino");

            return true;
        } catch(std::exception &e) {
            ROS_ERROR_STREAM("Servo controller error: Unable to parse limits from arduino -> closing");
            close(sockfd);
            return false;
        }

        }catch (std::exception &e) {
            ROS_ERROR_STREAM(" Hier muss error Code rein"); //#TODO
        
        return false;
     }
    
}

void ServoInterface::read() {
    // Calculating time since last update
    ros::Time current_time = ros::Time::now();
    last_run_period = current_time - last_run_time;
    last_run_time = current_time;

    
                    // Checking if the arduino is connected
                    // if(!arduino_serial.isOpen()) {
                    //     // Trying to reconnect
                    //     if(!try_open_serial_port()) {
                    //         return;
                    //     }
                    // }
    // Sending command to arduino
    sendto(sockfd, (const char *)p_Sender, strlen(p_Sender),MSG_CONFIRM, (const struct sockaddr *) &servaddr,len);
        //ROS_INFO("2 Message Sent");
    
                    // try {
                    //     arduino_serial.write("p\n");
                    //     arduino_serial.flush();
                    // } catch (serial::SerialException &e) {
                    //     ROS_ERROR_STREAM("Servo controller error: Unable to write to port " << arduino_serial.getPort());
                    //     ROS_ERROR_STREAM("Servo controller error: Error: " << e.what());
                    //     arduino_serial.close();
                    //     return;
                    // }
                    // Reading response from arduino
    received_message = recvfrom(sockfd, (char *)buffer, MAXLINE, MSG_WAITALL, ( struct sockaddr *) &servaddr, &len);
    buffer[received_message] = '\0';
        // ROS_INFO("3 Message Received");
        // ROS_INFO("4 Aktuelle Read Position %s",buffer);
    std::string response; 
                    //int size_arr = sizeof(buffer) / sizeof(char); 
	response = buffer;
                    // do {
                    //     try {
                    //         response += arduino_serial.readline();
                    //     } catch (serial::SerialException &e) {
                    //         ROS_ERROR_STREAM("Servo controller error: Unable to read from port " << arduino_serial.getPort());
                    //         ROS_ERROR_STREAM("Servo controller error: Error: " << e.what());
                    //         arduino_serial.close();
                    //         return;
                    //     }
                    // } while(arduino_serial.available() > 0);

    
    //Checking if response is long enough (p \r\n)
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
    position_joint_saturation_interface.enforceLimits(last_run_period);

                    // Checking if the arduino is connected
                    // Not retrying to connect here as we already tried in read()
                    // if(!arduino_serial.isOpen()) {
                    //     return;
                    // }

    // Not sending commands which already where before
   
    if(previous_command == servo_state.command) {
        ROS_INFO("Position = Position");
        return;
    }
    

    ROS_DEBUG_STREAM("Servo controller: Sending command to arduino: " << servo_state.command);

    // Remapping pwm value from radians input
    int command_value = top_limit - (int) round((servo_state.command / radian_limit) * (top_limit - bottom_limit));

    // Sending command to arduino
    std::string s_Sender = "s ";
    s_Sender += std::to_string(command_value);
    s_Sender +=("\n");
   
    
    char* write_pos_array = convert_to_char(s_Sender);
  

    sendto(sockfd, (const char *)write_pos_array, strlen(write_pos_array), 
        MSG_CONFIRM, (const struct sockaddr *) &servaddr,
            len);
        ROS_INFO("The write_pos_array contains %s",write_pos_array); 
        ROS_INFO("Write Message Sent");
                    // try {
                    //     arduino_serial.write(sstream.str());
                    //     arduino_serial.flush();
                    // } catch (serial::SerialException &e) {
                    //     ROS_ERROR_STREAM("Servo controller error: Failed to send command to arduino");
                    //     ROS_ERROR_STREAM("Servo controller error: Error: " << e.what());
                    //     arduino_serial.close();
                    //     return;
                    // }

    // Reading response from arduino
        received_message = recvfrom(sockfd, (char *)buffer, MAXLINE, 
                MSG_WAITALL, ( struct sockaddr *) &servaddr,
                &len);
        buffer[received_message] = '\0';
        
        std::string response; 
        //int size_arr = sizeof(buffer) / sizeof(char); 
	    response = buffer;
                    // do {
                    //     try {
                    //         response += arduino_serial.readline();
                    //     } catch (serial::SerialException &e) {
                    //         ROS_ERROR_STREAM("Servo controller error: Unable to read from port " << arduino_serial.getPort());
                    //         ROS_ERROR_STREAM("Servo controller error: Error: " << e.what());
                    //         arduino_serial.close();
                    //         return;
                    //     }
                    // } while(arduino_serial.available() > 0);

        // Removing trailing \r\n
        //response = response.substr(0, response.length() - 2);

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
        ROS_INFO("Position was changed successfully");
    }

    else {
        ROS_ERROR_STREAM("Servo controller error: Unknown error");
    }
}

char* ServoInterface::convert_to_char(std::string str){
  int length = str.length();
  
  // declaring character array (+1 for null terminator)
  char* char_array = new char[length + 1];
  // copying the contents of the
  // string to char array
  strcpy(char_array, str.c_str()); 
  return char_array;
}