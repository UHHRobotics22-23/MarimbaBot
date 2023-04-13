#ifndef MARIMBABOT_SERVO_INTERFACE_HPP
#define MARIMBABOT_SERVO_INTERFACE_HPP

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <serial/serial.h>


class ServoInterface : public hardware_interface::RobotHW {

private:
    struct ServoState {
        std::string name = "";
        double command = 0;
        double position = 0;
        double velocity = 0;
        double effort = 0;

        ServoState(const std::string &servo_name);
    };

public:
    ServoInterface(ros::NodeHandle& node_handle, std::string &device, int baud);

    void read();
    void write();

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
    serial::Serial arduino_serial;
    double previous_command = -1;
};

#endif // MARIMBABOT_SERVO_INTERFACE_HPP