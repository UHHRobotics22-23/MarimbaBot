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
    assert(original_length <= length && "Input plan must be shorter than the desired length");
    
    // Calculate the scaling factor
    double scaling_factor = length / original_length;

    // Copy the input plan
    moveit::planning_interface::MoveGroupInterface::Plan output_plan{input_plan};

    // Scale the time stamps in a functional way
    for(auto i = 0; i < output_plan.trajectory_.joint_trajectory.points.size(); i++)
    {
        output_plan.trajectory_.joint_trajectory.points[i].time_from_start *= scaling_factor;
    }

    // Interpolate trajectory
    output_plan = interpolate_plan(output_plan, 100);

    return output_plan;   
}


/**
 * @brief Interpolate a trajectory with a given number of points per second
 *
 * @param input_plan
 * @param points_per_second
 * @return moveit::planning_interface::MoveGroupInterface::Plan
 **/
moveit::planning_interface::MoveGroupInterface::Plan interpolate_plan(
    const moveit::planning_interface::MoveGroupInterface::Plan& input_plan,
    double points_per_second)
{
    assert(input_plan.trajectory_.joint_trajectory.points.size() > 0 && "Input plan must have at least one point");

    // Calculate the number of points
    double original_length = input_plan.trajectory_.joint_trajectory.points.back().time_from_start.toSec();
    int original_number_of_points = input_plan.trajectory_.joint_trajectory.points.size();
    int desired_number_of_points = original_length * points_per_second;
    int number_of_points_to_add = desired_number_of_points - original_number_of_points;

    // Create a linear interpolation of the input plan
    moveit::planning_interface::MoveGroupInterface::Plan interpolated_plan{input_plan};
    interpolated_plan.trajectory_.joint_trajectory.points.clear();
    for (auto i = 0; i < desired_number_of_points; i++)
    {
        double current_time = i * original_length / desired_number_of_points;

        // Find the two points to interpolate between
        int index_of_first_point = 0;
        int index_of_second_point = 0;
        for (auto j = 0; j < original_number_of_points; j++)
        {
            if (input_plan.trajectory_.joint_trajectory.points[j].time_from_start.toSec() > current_time)
            {
                index_of_second_point = j;
                index_of_first_point = std::max(0, j - 1);
                break;
            }
        }

        // Calculate the time difference between the two points
        double dt = input_plan.trajectory_.joint_trajectory.points[index_of_second_point].time_from_start.toSec() - input_plan.trajectory_.joint_trajectory.points[index_of_first_point].time_from_start.toSec();

        // Handle the case where dt is zero
        double interpolation_factor = 0;
        if (dt > 0)
        {
            // Calculate the interpolation factor
            interpolation_factor = (current_time - input_plan.trajectory_.joint_trajectory.points[index_of_first_point].time_from_start.toSec()) / dt;
        }

        // Interpolate between the two points
        trajectory_msgs::JointTrajectoryPoint interpolated_point;

        // Interpolate joint positions
        for (auto j = 0; j < input_plan.trajectory_.joint_trajectory.points[index_of_first_point].positions.size(); j++)
        {
            double first_point_position = input_plan.trajectory_.joint_trajectory.points[index_of_first_point].positions[j];
            double second_point_position = input_plan.trajectory_.joint_trajectory.points[index_of_second_point].positions[j];
            double interpolated_position = first_point_position + interpolation_factor * (second_point_position - first_point_position);
            interpolated_point.positions.push_back(interpolated_position);
        }

        // Add the interpolated point to the plan
        interpolated_point.time_from_start = ros::Duration(current_time);
        interpolated_plan.trajectory_.joint_trajectory.points.push_back(interpolated_point);
    }
    return interpolated_plan;
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

        // Get the position of the note in the planning frame by transforming a point in the origin of the note frame to the planning frame
        try
        {
            // Transform the point to the planning frame
            note_point = tf_buffer->transform(
            note_point,
            planning_frame,
            ros::Duration(1.0)
            );
            
            // Insert the cartesian point and original message into the struct
            hit_sequence_element.left_mallet_point = note_point;
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
    return cartesian_hit_sequence;
}


std::vector<CartesianHitSequenceElement> hit_sequence_absolute_to_relative(
            const std::vector<CartesianHitSequenceElement>& hit_sequence_absolute)
{
    // Create a vector of CartesianHitSequenceElements
    std::vector<CartesianHitSequenceElement> hit_sequence_relative;

    // Keep track of the current time
    double current_time = 0.0;

    // Add the goal point to the vector
    for (auto absolute_hit_sequence_element : hit_sequence_absolute)
    {
        // Add the struct to the vector
        auto relative_hit_sequence_element{absolute_hit_sequence_element};

        // Store the time of the current point
        auto current_element_time = absolute_hit_sequence_element.msg.start_time;

        // Set the time from start
        relative_hit_sequence_element.msg.start_time -= ros::Duration(current_time);

        // Update the current time
        current_time = current_element_time.toSec();

        // Add the struct to the vector
        hit_sequence_relative.push_back(relative_hit_sequence_element);
    }
    return hit_sequence_relative;
}


std::vector<CartesianHitSequenceElement> apply_chords(std::vector<CartesianHitSequenceElement> hits_relative)
{
    // Create a copy of the input vector
    auto hits_relative_with_chords{hits_relative};
    // Check if there are any hits with a (near) zero relative timing. 
    // These are chords and should be played at the same time using the second (right) mallet
    // Iterate over all hits
    for (auto hit_iter = hits_relative_with_chords.begin(); hit_iter != hits_relative_with_chords.end(); ++hit_iter) {
        // Check if the next hit has a relative timing of zero and we are not at the end of the vector
        if (hit_iter + 1 != hits_relative_with_chords.end() && (hit_iter + 1)->msg.start_time < ros::Time(0.05)) {
            // Get the chord hit points
            auto chord_hit_point_1 = hit_iter->left_mallet_point;
            auto chord_hit_point_2 = (hit_iter + 1)->left_mallet_point;
            // Add the right hit point to right mallet and the left one to the left mallet
            if(chord_hit_point_1.point.y < chord_hit_point_2.point.y) {
                hit_iter->right_mallet_point = chord_hit_point_1;
                hit_iter->left_mallet_point = chord_hit_point_2;
            } else {
                hit_iter->right_mallet_point = chord_hit_point_2;
                hit_iter->left_mallet_point = chord_hit_point_1;
            }
            // Remove the next hit as it is now a part of the current hit
            hits_relative_with_chords.erase(hit_iter + 1);
        }
    }
    return hits_relative_with_chords;
}

} // namespace marimbabot_planning
