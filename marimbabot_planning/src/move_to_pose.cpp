#include <bio_ik/bio_ik.h>
#include <boost/algorithm/string.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>
#include <regex>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

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
    ros::Duration time_from_start{0};
    for (int i = 1; i < plans.size(); i++)
    {
        time_from_start += plans[i].trajectory_.joint_trajectory.points.back().time_from_start;
        for (int j = 1; j < plans[i].trajectory_.joint_trajectory.points.size(); j++)
        {
            trajectory_msgs::JointTrajectoryPoint point;
            point = plans[i].trajectory_.joint_trajectory.points[j];
            point.time_from_start += time_from_start;
            plan.trajectory_.joint_trajectory.points.push_back(point);
        }
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
 * @brief Plan to Pose
 *
 * @param  start_state
 * @param  goal_pose
 * @param  move_group_interface
 * @return moveit::planning_interface::MoveGroupInterface::Plan
 * @throws std::runtime_error
 **/
moveit::planning_interface::MoveGroupInterface::Plan plan_to_pose(
    moveit::planning_interface::MoveGroupInterface& move_group_interface,
    const moveit_msgs::RobotState& start_state,
    geometry_msgs::PoseStamped goal_pose)
{
    // Initialize output plan
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    // Create robot state
    moveit::core::RobotState robot_state(move_group_interface.getRobotModel());
    robot_state.setToDefaultValues();

    // Set start state
    move_group_interface.setStartState(start_state);

    // Check if goal pose is in the same frame as the planning frame
    assert(goal_pose.header.frame_id == move_group_interface.getPlanningFrame());

    // Copy goal pose geometry_msgs::PoseStamped to tf2::Vector3 and tf2::Quaternion
    tf2::Vector3 goal_position(
        goal_pose.pose.position.x,
        goal_pose.pose.position.y,
        goal_pose.pose.position.z);
    tf2::Quaternion goal_orientation(
        goal_pose.pose.orientation.x,
        goal_pose.pose.orientation.y,
        goal_pose.pose.orientation.z,
        goal_pose.pose.orientation.w);

    // Use bio_ik to solve the inverse kinematics at the goal pose
    bio_ik::BioIKKinematicsQueryOptions ik_options;
    ik_options.replace = true;
    ik_options.return_approximate_solution = false;
    ik_options.goals.emplace_back(new bio_ik::PoseGoal("diana_gripper/tcp", goal_position, goal_orientation));
    if(!robot_state.setFromIK(
        move_group_interface.getRobotModel()->getJointModelGroup(move_group_interface.getName()),
        goal_pose.pose /* this is ignored with replace = true */,
        0.0,
        moveit::core::GroupStateValidityCallbackFn(),
        ik_options))
    {
        throw std::runtime_error("IK for approach pose failed");
    }

    // Plan to the goal pose (but in joint space)
    move_group_interface.setJointValueTarget(robot_state);

    // Plan
    if(!move_group_interface.plan(plan))
    {
        throw std::runtime_error("Approach plan failed");
    };

    return plan;
}


/**
 * @brief hit a given point in cartesian space
 *
 * @param start_state
 * @param move_group_interface
 * @param pose
 * @return moveit::planning_interface::MoveGroupInterface::Plan
**/

moveit::planning_interface::MoveGroupInterface::Plan hit_point(
    moveit::planning_interface::MoveGroupInterface& move_group_interface,
    const moveit_msgs::RobotState& start_state,
    geometry_msgs::PoseStamped pose)
{

    moveit::core::RobotState robot_state(move_group_interface.getRobotModel());
    robot_state.setToDefaultValues();
    moveit::core::robotStateMsgToRobotState(start_state, robot_state);


    // Calculate approach pose
    geometry_msgs::PoseStamped approach_pose{pose};
    approach_pose.pose.position.z += 0.02;
    // Calculate retreat pose
    geometry_msgs::PoseStamped retreat_pose{approach_pose};

    // Calculate approach trajectory
    auto approach_plan = plan_to_pose(move_group_interface, start_state, approach_pose);

    // Calculate down trajectory
    auto down_plan = plan_to_pose(move_group_interface, get_robot_state_after_plan(approach_plan), pose);

    // Calculate retreat trajectory
    auto retreat_plan = plan_to_pose(move_group_interface, get_robot_state_after_plan(down_plan), retreat_pose);

    // Concatinate trajectories
    auto plan = concatinated_plan({approach_plan, down_plan, retreat_plan});

    return plan;
}

/**
 * @brief Hit a sequence of points in cartesian space
 *
 * @param move_group_interface
 * @param start_state
 * @param poses
 * @return moveit::planning_interface::MoveGroupInterface::Plan
**/
moveit::planning_interface::MoveGroupInterface::Plan hit_points(
    moveit::planning_interface::MoveGroupInterface& move_group_interface,
    const moveit_msgs::RobotState& start_state,
    std::vector<geometry_msgs::PoseStamped> poses)
{
    // Assert that there is at least one hit_point with an nice error message
    assert(poses.size() > 0 && "There must be at least one hit_point");

    // Calculate hit trajectory
    auto hit_plan = hit_point(
        move_group_interface,
        start_state,
        poses.front()
        );

    // Call hit_points recursively for all remaining hit_points
    if(poses.size() > 1)
    {
        auto remaining_hit_plan = hit_points(
            move_group_interface,
            get_robot_state_after_plan(hit_plan),
            std::vector<geometry_msgs::PoseStamped>(poses.begin() + 1, poses.end())
            );
        hit_plan = concatinated_plan({hit_plan, remaining_hit_plan});
    }

    return hit_plan;
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

    return output_plan;
}


/**
 * @brief Convert lilypond sequence to cartesian poses and times
 *
 * @param tf_buffer
 * @param planning_frame
 * @param lilypond
 * @param tempo (default: 120.0)
 * @return std::vector<std::pair<geometry_msgs::PoseStamped, double>>
 **/
std::vector<std::pair<geometry_msgs::PoseStamped, double>> lilypond_to_cartesian(
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::string& planning_frame,
    std::string lilypond,
    double tempo = 120.0)
{
    // The lilypond sequence is a string of notes and looks like this: "c'4 d''4 e4 f'4 g'16"
    // The notes are separated by spaces and the duration is given by the number after the note
    // The note is given by the letter and the octave by the ' and the number after the letter

    // Split the string into a vector of notes
    std::vector<std::string> notes;
    boost::split(notes, lilypond, boost::is_any_of(" "));

    // Initialize the vector of cartesian poses and times
    std::vector<std::pair<geometry_msgs::PoseStamped, double>> hits;

    double time = 0.0;

    // Each key on the marimba has its own tf frame called something like "bar_F#5" or "bar_C4"
    // Iterate over all notes and calculate the cartesian pose for each note
    for(auto note : notes)
    {
        // Use regex to check that the note is valid and parsable
        std::regex note_regex("^[a-gr][']*[0-9]+$");
        if(!std::regex_match(note, note_regex))
        {
            ROS_WARN_STREAM("Note '" << note << "' is not valid");
            continue;
        }

        // Get the first letter
        std::string note_letter = note.substr(0, 1);

        // Check if note is a rest
        if (note_letter == "r")
        {
            // If the note is a rest, we don't need to calculate the cartesian pose and instead only increment the time
            time += 60 / std::stod(note.substr(1)) / tempo;
        }
        else
        {
            // Count the number of ' in the string to get the octave
            int octave = std::count(note.begin(), note.end(), '\'');

            // Get the duration of the note by getting the substring starting at 1 + octave
            double duration = 60 / std::stod(note.substr(1 + octave)) / tempo;

            // Capitalize the note letter
            note_letter[0] = std::toupper(note_letter[0]);

            // Map the note letter and octave to the corresponding tf frame
            std::string note_frame = "bar_" + note_letter + std::to_string(octave + 4);

            // Get the cartesian pose of the note
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = note_frame;  // The frame of the note
            pose.header.stamp = ros::Time(0);  // Use the latest available transform
            pose.pose.orientation.w = 1.0;  // The orientation of the note

            try
            {
                // Transform the pose to the planning frame
                auto transformed_pose = tf_buffer->transform(
                    pose,
                    planning_frame,
                    ros::Duration(1.0)
                    );

                // Add the cartesian pose and time to the vector
                hits.push_back(std::make_pair(transformed_pose, time));
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("Failed to get the transformation from the robot base to the note frame on the marimba! Error: %s", ex.what());
                ROS_WARN("Skipping note %s", note.c_str());
                continue;
            }

            // Increment the time
            time += duration;
        }
    }

    return hits;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "marimba_move");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Create tf2 listener
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    static const std::string PLANNING_GROUP = "arm_with_tcp";
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
    move_group_interface.setPlanningPipelineId("pilz_industrial_motion_planner");
    move_group_interface.setPlannerId("PTP");
    move_group_interface.setMaxVelocityScalingFactor(0.1);
    move_group_interface.setMaxAccelerationScalingFactor(0.1);
    move_group_interface.startStateMonitor();

    const moveit::core::JointModelGroup* joint_model_group =
        move_group_interface.getRobotModel()->getJointModelGroup(PLANNING_GROUP);

    // Lookup the world to tcp transform
    geometry_msgs::TransformStamped transformStamped;
    try
    {
        transformStamped = tfBuffer.lookupTransform("world", "diana_gripper/tcp", ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s",ex.what());
    }

    sensor_msgs::JointState home_joints;
    //TODO : turn this into moveit group_state
    home_joints.name = { "joint_1","joint_2","joint_3","joint_4","joint_5","joint_6","joint_7" };
    home_joints.position = {-1.0764353166380913, 0.5625393633338827, 0.31906783564462415, 2.316970072925777, 0.042747545531845337, 1.0091591209316584, 1.4176498139743448};

    move_group_interface.setJointValueTarget(home_joints);
    move_group_interface.move();

    // Convert transform to pose
    geometry_msgs::PoseStamped pose;

    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = transformStamped.header.frame_id;
    pose.pose.position.x = transformStamped.transform.translation.x;
    pose.pose.position.y = transformStamped.transform.translation.y;
    pose.pose.position.z = transformStamped.transform.translation.z;
    pose.pose.orientation.x = transformStamped.transform.rotation.x;
    pose.pose.orientation.y = transformStamped.transform.rotation.y;
    pose.pose.orientation.z = transformStamped.transform.rotation.z;
    pose.pose.orientation.w = transformStamped.transform.rotation.w;

    // Print pose
    ROS_INFO_STREAM("Pose: " << pose);

    auto current_state = move_group_interface.getCurrentState();
    //convert to moveit message
    moveit_msgs::RobotState start_state;
    moveit::core::robotStateToRobotStateMsg(*current_state, start_state);

    // Hit a sequence of points in cartesian space left and right of "pose"

    // Define hit points
    std::vector<geometry_msgs::PoseStamped> hit_poses;
    for (auto offset : {-0.2, -0.1, 0.0, 0.1, 0.2})
    {
        geometry_msgs::PoseStamped hit_point{pose};
        hit_point.pose.position.y += offset;
        hit_poses.push_back(hit_point);
    }

    auto hit_plan = hit_points(move_group_interface, start_state, hit_poses);

    // Publish the plan for rviz
    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;
    moveit_msgs::RobotTrajectory trajectory;
    trajectory.joint_trajectory = hit_plan.trajectory_.joint_trajectory;
    display_trajectory.trajectory_start = hit_plan.start_state_;
    display_trajectory.trajectory.push_back(trajectory);
    display_publisher.publish(display_trajectory);

    // Execute the plan
    //move_group_interface.execute(hit_plan);
    ros::waitForShutdown();
    return 0;
}
