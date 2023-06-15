#ifndef MARIMBABOT_SERVO_INTERFACE_HPP
#define MARIMBABOT_SERVO_INTERFACE_HPP

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <iostream>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <bits/stdc++.h>
#include <stdlib.h>
#include <string.h>


#define GETSOCKETERRNO() (errno)
#define PORT     8888
#define MAXLINE 1024
//#include <serial/serial.h>


class ServoInterface : public hardware_interface::RobotHW {

private:
    struct ServoState {
        std::string name = "";
        double command = -1;    // -1 means no command yet
        double position = 0;
        double velocity = 0;
        double effort = 0;
        

        ServoState(const std::string &servo_name);
    };

public:
    ServoInterface(ros::NodeHandle& node_handle, std::string &address, int port); //device

    void read();
    void write();
    void initialize();

    const ros::Time& get_time() { return last_run_time; };
    const ros::Duration& get_period() { return last_run_period; };

private:
    hardware_interface::JointStateInterface joint_state_interface;
    hardware_interface::PositionJointInterface position_joint_interface;
    joint_limits_interface::PositionJointSaturationInterface position_joint_saturation_interface;
    joint_limits_interface::JointLimits servo_limits;
    ServoState servo_state;

    ros::Time last_run_time;
    ros::Duration last_run_period;
    //serial::Serial arduino_serial;
    int last_arduino_error = 0;
    double previous_command = -1;
    int top_limit = -1;
    int bottom_limit = -1;
    int resolution = -1;
    double radian_limit = -1;

    int sockfd;
    char buffer[MAXLINE];
    const char *l_Sender = "l\n";
    const char *p_Sender = "p\n";
    
    struct sockaddr_in servaddr, cliaddr;
  
    socklen_t len;
    int received_message;

    bool try_open_udp_port();
    char* convert_to_char(std::string str);
    
    
};

#endif // MARIMBABOT_SERVO_INTERFACE_HPP