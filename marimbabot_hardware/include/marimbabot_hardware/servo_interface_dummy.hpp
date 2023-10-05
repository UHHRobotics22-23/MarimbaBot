/*
 * Dummy servo interface for testing and simulation for one servo.
 * Tries to emulate the behaviour of the serial servo interface.
 */
#ifndef MARIMBABOT_SERVO_INTERFACE_HPP
#define MARIMBABOT_SERVO_INTERFACE_HPP

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>


// Class definition of the hardware interface for a single dummy servo motor for use with ros_controllers
class ServoInterface : public hardware_interface::RobotHW {

private:
    // The struct to hold the state of the servo motor which will be registered to the hardware interface
    struct ServoState {
        std::string name = "";
        double command = -1;    // -1 means no command yet
        double position = 0;
        double velocity = 0;
        double effort = 0;

        ServoState(const std::string &servo_name);
    };

public:
    ServoInterface(ros::NodeHandle& node_handle);

    // The read function is called by the main node to read the state of the servo motor
    void read();
    // The write function is called by the main node to write the command to the servo motor
    void write();
    // The initialize function is used to ensure a read is performed before any other action
    void initialize();

    // The get_time and get_period functions are used by the controller manager to keep track of time
    const ros::Time& get_time() { return last_run_time; };
    const ros::Duration& get_period() { return last_run_period; };

private:
    // The joint state interface is used to register the servo state to the controller manager
    hardware_interface::JointStateInterface joint_state_interface;
    // The position joint interface is used to register the servo command to the controller manager
    hardware_interface::PositionJointInterface position_joint_interface;
    // The joint limits are used to ensure the servo command is within the limits of the servo
    joint_limits_interface::PositionJointSaturationInterface position_joint_saturation_interface;
    joint_limits_interface::JointLimits servo_limits;
    // The state of the servo motor
    ServoState servo_state;

    // The time of the last read and the period between reads
    ros::Time last_run_time;
    ros::Duration last_run_period;
    // The previous command sent to the servo motor
    double previous_command = -1;
    // The limits of the servo as given by the l command
    int top_limit = -1;
    int bottom_limit = -1;
    int resolution = -1;
    // The limit of the servo mapped to radians from 0 to radian_limit
    double radian_limit = -1;
};

#endif // MARIMBABOT_SERVO_INTERFACE_HPP