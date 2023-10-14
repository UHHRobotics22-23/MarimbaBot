#include <boost/algorithm/string.hpp>
#include <boost/optional.hpp>
#include <geometry_msgs/PointStamped.h>
#include <marimbabot_msgs/HitSequenceAction.h>
#include <moveit_msgs/RobotState.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>


namespace marimbabot_planning
{

/**
 * @brief Custom exception for the case that a plan failed
 * 
 **/
class PlanFailedException : public std::runtime_error 
{ using std::runtime_error::runtime_error; };

/**
 * @brief Custom exception for the case that a plan failed because of IK issues
 * 
 **/
class IKFailedException : public PlanFailedException 
{ using PlanFailedException::PlanFailedException; };

/**
 * @brief Struct containing a hit point in cartesian space and the corresponding HitSequenceElement
 * 
 **/
struct CartesianHitSequenceElement
{
    geometry_msgs::PointStamped left_mallet_point;
    boost::optional<geometry_msgs::PointStamped> right_mallet_point;
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
 * @brief Interpolate a trajectory with a given number of points per second
 *
 * @param input_plan
 * @param points_per_second
 * @return moveit::planning_interface::MoveGroupInterface::Plan
 **/
moveit::planning_interface::MoveGroupInterface::Plan interpolate_plan(
    const moveit::planning_interface::MoveGroupInterface::Plan& input_plan,
    double points_per_second);

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


/**
 * @brief Convert the timing information of a hit sequence from absolute to relative timings
 * 
 * @param hit_sequence
 * @return std::vector<CartesianHitSequenceElement>
 **/
std::vector<CartesianHitSequenceElement> hit_sequence_absolute_to_relative(
    const std::vector<CartesianHitSequenceElement>& hit_sequence);


/**
 * @brief Finds all chords in a sequence of notes and assigns the second malllet if one is detected
 * 
 * @param hits_relative Vector of notes with relative timing
 * @return std::vector<CartesianHitSequenceElement> Vector of notes with relative timing, both mallets are assigned if a chord is detected. This may be shorter than the input vector.
*/
std::vector<CartesianHitSequenceElement> apply_chords(std::vector<CartesianHitSequenceElement> hits_relative);
} // namespace marimbabot_planning