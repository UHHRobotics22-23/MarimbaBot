#include <geometry_msgs/PointStamped.h>
#include <marimbabot_msgs/HitSequenceAction.h>
#include <moveit_msgs/RobotState.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>


namespace marimbabot_planning
{

struct CartesianHitSequenceElement
{
    geometry_msgs::PointStamped point;
    marimbabot_msgs::HitSequenceElement msg;
};


/**
 * @brief concatinates a vector of n plans (n>0) into one plan
 *
 * @param plans
 * @return moveit::planning_interface::MoveGroupInterface::Plan
**/
moveit::planning_interface::MoveGroupInterface::Plan concatinated_plan(std::vector<moveit::planning_interface::MoveGroupInterface::Plan> plans);


/**
 * @brief get robot state after plan
 *
 * @param  plan
 * @return moveit_msgs::RobotState
**/
moveit_msgs::RobotState get_robot_state_after_plan(moveit::planning_interface::MoveGroupInterface::Plan plan);


/**
 * @brief Slow down a trajectory to a given length. No speedup is possible.
 *
 * @param input_plan
 * @param length
 * @return moveit::planning_interface::MoveGroupInterface::Plan
 **/
moveit::planning_interface::MoveGroupInterface::Plan slow_down_plan(
    const moveit::planning_interface::MoveGroupInterface::Plan& input_plan,
    double length);


/**
 * @brief Convert a hit sequence elements to a vector of HitSequenceElement structs containing the points 
 * 
 * @param hit_sequence
 * @param tf_buffer
 * @param planning_frame
 * @return std::vector<CartesianHitSequenceElement>
 **/
std::vector<CartesianHitSequenceElement> hit_sequence_to_points(
    const std::vector<marimbabot_msgs::HitSequenceElement>& hit_sequence,
    std::string planning_frame,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer);

} 
