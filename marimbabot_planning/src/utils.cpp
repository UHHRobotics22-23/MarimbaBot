#include "marimbabot_planning/utils.h"
#include <cmath>

namespace marimbabot_planning
{

/**
 * @brief concatinates a vector of n plans (n>0) into one plan
 *
 * @param plans
 * @return moveit::planning_interface::MoveGroupInterface::Plan
**/
moveit::planning_interface::MoveGroupInterface::Plan concatinated_plan(std::vector<moveit::planning_interface::MoveGroupInterface::Plan> plans)
{
    // assert at least one plan
    assert(plans.size() > 0);

    moveit::planning_interface::MoveGroupInterface::Plan plan{plans[0]};
    ros::Duration time_from_start{plans[0].trajectory_.joint_trajectory.points.back().time_from_start};
    for (int i = 1; i < plans.size(); i++)
    {
        for (int j = 1; j < plans[i].trajectory_.joint_trajectory.points.size(); j++)
        {
            trajectory_msgs::JointTrajectoryPoint point;
            point = plans[i].trajectory_.joint_trajectory.points[j];
            point.time_from_start += time_from_start;
            plan.trajectory_.joint_trajectory.points.push_back(point);
        }
        time_from_start += plans[i].trajectory_.joint_trajectory.points.back().time_from_start;
    }
    return plan;
}


/**
 * @brief get robot state after plan
 *
 * @param  plan
 * @return moveit_msgs::RobotState
**/
moveit_msgs::RobotState get_robot_state_after_plan(moveit::planning_interface::MoveGroupInterface::Plan plan)
{
    moveit_msgs::RobotState state_after_plan {plan.start_state_};

    // Check if plan is empty
    if (plan.trajectory_.joint_trajectory.points.size() != 0)
    {
        // Get last point of plan
        trajectory_msgs::JointTrajectoryPoint last_point;
        last_point = plan.trajectory_.joint_trajectory.points.back();

        // Set joint positions of state after plan
        for (int i = 0; i < last_point.positions.size(); i++)
        {
            state_after_plan.joint_state.position[i] = last_point.positions[i];
            state_after_plan.joint_state.velocity[i] = 0.0;

        }
    }
    return state_after_plan;
}


/**
 * @brief Slow down a trajectory to a given length. No speedup is possible.
 *
 * @param input_plan
 * @param length
 * @return moveit::planning_interface::MoveGroupInterface::Plan
 **/

moveit::planning_interface::MoveGroupInterface::Plan slow_down_plan(
    const moveit::planning_interface::MoveGroupInterface::Plan& input_plan,
    double length)
{
    assert(input_plan.trajectory_.joint_trajectory.points.size() > 0 && "Input plan must have at least one point");

    // Get the time from start of the last point
    double original_length = input_plan.trajectory_.joint_trajectory.points.back().time_from_start.toSec();

    // Assert that the input plan is shorter than the desired length
    //assert(original_length <= length && "Input plan must be shorter than the desired length");
    ROS_INFO("Original length: %f", original_length);
    ROS_INFO("Desired length: %f", length);
    // Calculate the scaling factor
    //double scaling_factor = length / original_length;

    double scaling_factor = pow(original_length,length);
    ROS_INFO("Scaling factor: %f", scaling_factor);

    // Copy the input plan
    moveit::planning_interface::MoveGroupInterface::Plan output_plan{input_plan};

    // Scale the time stamps in a functional way
    for(auto i = 0; i < output_plan.trajectory_.joint_trajectory.points.size(); i++)
    {
        output_plan.trajectory_.joint_trajectory.points[i].time_from_start *= scaling_factor;
    }

    return output_plan;   
}


std::vector<CartesianHitSequenceElement> hit_sequence_to_points(
            const std::vector<marimbabot_msgs::HitSequenceElement>& hit_sequence,
            std::string planning_frame,
            std::shared_ptr<tf2_ros::Buffer> tf_buffer)
{
    // Create a vector of CartesianHitSequenceElements
    std::vector<CartesianHitSequenceElement> cartesian_hit_sequence;

    // Add the goal point to the vector
    for (auto point : hit_sequence)
    {
        // Map the note letter and octave to the corresponding tf frame
        std::string note_frame = "bar_" + point.tone_name + std::to_string(point.octave);

        // Get the cartesian point of the note
        geometry_msgs::PointStamped note_point;
        note_point.header.frame_id = note_frame;  // The frame of the note
        note_point.header.stamp = ros::Time(0);  // Use the latest available transform
        // Add the cartesian point and time to the vector
        CartesianHitSequenceElement hit_sequence_element;

        // skip frame transform and stay at the same position if tone_name is "rest"
        if (point.tone_name == "r")
        {
            note_point.point.z += 0.04;
            hit_sequence_element.point = note_point;
            hit_sequence_element.msg = point;
            cartesian_hit_sequence.push_back(hit_sequence_element);
        }
        else
        {
            try
            {
                // Transform the point to the planning frame
                note_point = tf_buffer->transform(
                note_point,
                planning_frame,
                ros::Duration(1.0)
                );

                // Add temporary z offset
                note_point.point.z += 0.04; 
                hit_sequence_element.point = note_point;
                hit_sequence_element.msg = point;
                
                // Add the struct to the vector
                cartesian_hit_sequence.push_back(hit_sequence_element);
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("Failed to get the transformation from the robot base to the note frame on the marimba! Error: %s", ex.what());
                ROS_WARN("Skipping note %s", point.tone_name.c_str());
                continue;
            }
        }
        
    }
    return cartesian_hit_sequence;
}

} // namespace marimbabot_planning

