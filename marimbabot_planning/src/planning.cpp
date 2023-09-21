#include "marimbabot_planning/planning.h"

namespace marimbabot_planning
{

/**
 * @brief Construct a new Planning:: Planning object
 *
**/
Planning::Planning(const std::string planning_group) :
    nh_{},
    tf_listener_{*tf_buffer_},
    move_group_interface_{planning_group},
    action_server_{
        nh_,
        "hit_sequence",
        boost::bind(&Planning::action_server_callback, this, _1),
        false},
    interactive_marker_server_{"mallet_markers"}
{
    // Set planning pipeline and planner
    move_group_interface_.setPlanningPipelineId("ompl");    
     //("pilz_industrial_motion_planner");
    move_group_interface_.setPlannerId("PTP");
    move_group_interface_.startStateMonitor();
    // Move to home position
    go_to_home_position();
    // Timer callback that moves back to the home position if no action is active
    auto timer_callback = [this](const ros::TimerEvent& event) {
        // Check if an action in not active and if the last action was more than 5 seconds ago
        if (!action_server_.isActive() && ros::Time::now() - last_action_time_ > ros::Duration(1.0))
        {
            // Check if marker controls are active
            if(marker_controls_active_)
            {
                try {
                    // Set the max velocity and acceleration scaling factors
                    move_group_interface_.setMaxVelocityScalingFactor(1.0);
                    move_group_interface_.setMaxAccelerationScalingFactor(1.0);

                    auto current_state = move_group_interface_.getCurrentState();
                    //convert to moveit message
                    moveit_msgs::RobotState start_state;
                    moveit::core::robotStateToRobotStateMsg(*current_state, start_state);

                    // Create a keyframe with the new position
                    DoubleMalletKeyframe keyframe;
                    keyframe.left_mallet_active = true;
                    keyframe.right_mallet_active = true;
                    keyframe.left_mallet_position = left_mallet_marker_state_;
                    keyframe.right_mallet_position = right_mallet_marker_state_;
                    keyframe.duration = 0.5;

                    // Calculate the joint space plan to the mallet keyframe
                    auto plan_to_keyframe = plan_to_mallet_position(start_state, keyframe);

                    // Publish the plan for rviz
                    moveit_msgs::DisplayTrajectory display_trajectory;
                    moveit_msgs::RobotTrajectory trajectory;
                    trajectory.joint_trajectory = plan_to_keyframe.trajectory_.joint_trajectory;
                    display_trajectory.trajectory_start = plan_to_keyframe.start_state_;
                    display_trajectory.trajectory.push_back(trajectory);
                    trajectory_publisher_.publish(display_trajectory);

                    // Execute the plan
                    auto status = move_group_interface_.execute(plan_to_keyframe);

                    // Print error if the execution failed
                    if (status != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
                        ROS_ERROR_STREAM("Execution failed: " << status);
                    }
                } catch (PlanFailedException& e) {
                    ROS_ERROR_STREAM("Planning failed: " << e.what());
                }
            } else {
                // Move back to the home position
                // go_to_home_position();  TODO reanable this
            }
        }
    };

    // Remove existing interactive markers
    interactive_marker_server_.erase("left_mallet_marker");
    interactive_marker_server_.erase("right_mallet_marker");

    // Create two interactive markers for the mallets
    visualization_msgs::InteractiveMarker left_mallet_marker, right_mallet_marker;

    // Get the current position of the mallets in the planning frame
    auto current_left_mallet_position = move_group_interface_.getCurrentPose("mallet_head_1").pose.position;
    auto current_right_mallet_position = move_group_interface_.getCurrentPose("mallet_head_2").pose.position;

    // Initialize marker states
    left_mallet_marker_state_.header.frame_id = move_group_interface_.getPlanningFrame();
    left_mallet_marker_state_.point.x = current_left_mallet_position.x;
    left_mallet_marker_state_.point.y = current_left_mallet_position.y;
    left_mallet_marker_state_.point.z = current_left_mallet_position.z;
    right_mallet_marker_state_.header.frame_id = move_group_interface_.getPlanningFrame();
    right_mallet_marker_state_.point.x = current_right_mallet_position.x;
    right_mallet_marker_state_.point.y = current_right_mallet_position.y;
    right_mallet_marker_state_.point.z = current_right_mallet_position.z;

    // Initialize the interactive markers
    left_mallet_marker.header.frame_id = move_group_interface_.getPlanningFrame();
    left_mallet_marker.name = "left_mallet_marker";
    left_mallet_marker.description = "Left Mallet";
    left_mallet_marker.scale = 0.2;
    left_mallet_marker.pose.position = left_mallet_marker_state_.point;
    right_mallet_marker.header.frame_id = move_group_interface_.getPlanningFrame();
    right_mallet_marker.name = "right_mallet_marker";
    right_mallet_marker.description = "Right Mallet";
    right_mallet_marker.scale = 0.2;
    right_mallet_marker.pose.position = right_mallet_marker_state_.point;


    // Create a control for the mallets
    visualization_msgs::InteractiveMarkerControl left_mallet_control, right_mallet_control;
    left_mallet_control.always_visible = true;
    right_mallet_control.always_visible = true;
    left_mallet_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_3D;
    right_mallet_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_3D;
    left_mallet_control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
    right_mallet_control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
    left_mallet_control.name = "left_mallet_control";
    right_mallet_control.name = "right_mallet_control";

    // Create a marker for the mallets
    double mallet_marker_scale = 0.05;
    visualization_msgs::Marker left_mallet_marker_marker, right_mallet_marker_marker;
    left_mallet_marker_marker.type = visualization_msgs::Marker::SPHERE;
    right_mallet_marker_marker.type = visualization_msgs::Marker::SPHERE;
    left_mallet_marker_marker.scale.x = mallet_marker_scale;
    left_mallet_marker_marker.scale.y = mallet_marker_scale;
    left_mallet_marker_marker.scale.z = mallet_marker_scale;
    right_mallet_marker_marker.scale.x = mallet_marker_scale;
    right_mallet_marker_marker.scale.y = mallet_marker_scale;
    right_mallet_marker_marker.scale.z = mallet_marker_scale;
    left_mallet_marker_marker.color.r = 1.0;
    left_mallet_marker_marker.color.a = 1.0;
    right_mallet_marker_marker.color.b = 1.0;
    right_mallet_marker_marker.color.a = 1.0;

    // Add the marker to the control
    left_mallet_control.markers.push_back(left_mallet_marker_marker);
    right_mallet_control.markers.push_back(right_mallet_marker_marker);

    // Add the control to the interactive marker
    left_mallet_marker.controls.push_back(left_mallet_control);
    right_mallet_marker.controls.push_back(right_mallet_control);

    // Add the interactive marker to the server
    interactive_marker_server_.insert(left_mallet_marker, boost::bind(&Planning::process_interactive_marker_feedback, this, _1));
    interactive_marker_server_.insert(right_mallet_marker, boost::bind(&Planning::process_interactive_marker_feedback, this, _1));
    interactive_marker_server_.applyChanges();

    // Create timer that checks if an action is active and moves back to the home position if not
    auto timer = nh_.createTimer(ros::Duration(0.5), timer_callback);
    // Start action server
    action_server_.start();
    // Wait for shutdown
    ros::waitForShutdown();
}


/**
 * @brief Processes feedback from the interactive markers
 *
 * @param feedback
 */
void Planning::process_interactive_marker_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    ROS_WARN_STREAM("Interactive marker feedback: " << feedback->marker_name);
    // Log position of the marker
    ROS_WARN_STREAM("Position: " << feedback->pose.position.x << ", " << feedback->pose.position.y << ", " << feedback->pose.position.z);
    // Insert new pose into the correct marker state variable
    if (feedback->marker_name == "left_mallet_marker") {
        left_mallet_marker_state_.header = feedback->header;
        left_mallet_marker_state_.point = feedback->pose.position;
    } else if (feedback->marker_name == "right_mallet_marker") {
        right_mallet_marker_state_.header = feedback->header;
        right_mallet_marker_state_.point = feedback->pose.position;
    }
    // Set the marker controls active
    marker_controls_active_ = true;
}


/**
 * @brief Move the robot to its idle/home position
*/
void Planning::go_to_home_position()
{
    // Set slow velocity and acceleration scaling factor as speed is not important
    move_group_interface_.setMaxVelocityScalingFactor(0.05);
    move_group_interface_.setMaxAccelerationScalingFactor(0.05);
    // Set home position as target
    move_group_interface_.setNamedTarget("marimbabot_home");
    move_group_interface_.move();
}


/**
 * @brief Calculate a plan so that the mallet (end effector) is at a given point in cartesian space
 *goal
 * @param  start_state
 * @param  goal_point
 * @return moveit::planning_interface::MoveGroupInterface::Plan
 * @throws PlanFailedException, IKFailedException
 **/
moveit::planning_interface::MoveGroupInterface::Plan Planning::plan_to_mallet_position(
    const moveit_msgs::RobotState& start_state,
    DoubleMalletKeyframe goal)
{
    // Initialize output plan
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    // Get the joint values of the marimbabot_home position and set them as the seed state
    // Get the marimbabot_home position from moveit
    auto marimbabot_home = move_group_interface_.getNamedTargetValues("marimbabot_home");

    // Create vector of joint names based on the marimbabot_home position 
    std::vector<std::string> marimbabot_home_joint_names;
    std::vector<double> marimbabot_home_joint_values;
    for (auto joint : marimbabot_home) {
        marimbabot_home_joint_names.push_back(joint.first);
        marimbabot_home_joint_values.push_back(joint.second);
    }

    // Create robot state
    moveit::core::RobotState robot_state(move_group_interface_.getRobotModel());
    robot_state.setToDefaultValues();
    robot_state.setVariablePositions(marimbabot_home_joint_names, marimbabot_home_joint_values);
    robot_state.update();

    // Set start state
    move_group_interface_.setStartState(start_state);

    // Check if goal point is in the same frame as the planning frame
    assert(!goal.left_mallet_active || (goal.left_mallet_position.header.frame_id == move_group_interface_.getPlanningFrame()));
    assert(!goal.right_mallet_active || (goal.right_mallet_position.header.frame_id == move_group_interface_.getPlanningFrame()));

    // Check if none of the mallets is defined else print error
    assert(goal.left_mallet_active || goal.right_mallet_active);

    // Copy goals from geometry_msgs::PointStamped to tf2::Vector3
    tf2::Vector3 left_mallet_goal_position(
        goal.left_mallet_position.point.x,
        goal.left_mallet_position.point.y,
        goal.left_mallet_position.point.z + 0.02);

    tf2::Vector3 right_mallet_goal_position(
        goal.right_mallet_position.point.x,
        goal.right_mallet_position.point.y,
        goal.right_mallet_position.point.z + 0.02);

    // Use bio_ik to solve the inverse kinematics at the goal point
    bio_ik::BioIKKinematicsQueryOptions ik_options;
    ik_options.replace = true;
    ik_options.return_approximate_solution = true; // Activate for debugging if you get an error

    // Define lambda function for link on plane constraint
    auto link_on_plane_constraint = [](tf2::Vector3 plane_point) -> std::function<double(const tf2::Vector3&, const tf2::Quaternion&)>
    {
        return [plane_point](const tf2::Vector3& position, const tf2::Quaternion& orientation) -> double
        {
            tf2::Vector3 plane_normal(0.0, 0.0, 1.0);
            tf2::Vector3 plane_to_position = position - plane_point;
            double signed_dist = plane_to_position.dot(plane_normal);
            // Take the squared value of the signed distance
            return std::pow(signed_dist, 2);
        };
    };

    double default_mallet_height = 1.0;

    // Set left mallet goal if it is defined
    if (goal.left_mallet_active) {
        // Set goal position for left mallet
        ik_options.goals.emplace_back(new bio_ik::PositionGoal("mallet_head_1", left_mallet_goal_position));
    } else {
        // Set link on plane constraint for left mallet
        ik_options.goals.emplace_back(new bio_ik::LinkFunctionGoal(
            "mallet_head_1",
            link_on_plane_constraint(tf2::Vector3(0.0, 0.0, default_mallet_height))));
    }

    // Set right mallet goal if it is defined
    if (goal.right_mallet_active) {
        // Set goal position for right mallet
        ik_options.goals.emplace_back(new bio_ik::PositionGoal("mallet_head_2", right_mallet_goal_position));
    } else {
        // Set link on plane constraint for right mallet
        ik_options.goals.emplace_back(new bio_ik::LinkFunctionGoal(
            "mallet_head_2",
            link_on_plane_constraint(tf2::Vector3(0.0, 0.0, default_mallet_height))));
    }

    // Set auxilary goals if we play with only one mallet
    if (goal.left_mallet_active != goal.right_mallet_active) {
        // Keep the double mallet joint at 70 degrees
        ik_options.goals.emplace_back(new bio_ik::JointVariableGoal("mallet_finger", 60.0 * M_PI / 180.0));
        // Add joint variable goal for the wrist joint to avoid unnecessary rotations
        ik_options.goals.emplace_back(new bio_ik::JointVariableGoal("ur5_wrist_3_joint", 0.0));
    }

    // Add link on plane constraint to ik_options
    ik_options.goals.emplace_back(new bio_ik::LinkFunctionGoal("ur5_wrist_1_link", link_on_plane_constraint(tf2::Vector3(0.0, 0.0, 1.3))));

    // Add minimal displacement constraint to ik_options
    ik_options.goals.emplace_back(new bio_ik::MinimalDisplacementGoal());

    // Create dummy goal pose
    geometry_msgs::Pose dummy_goal_pose;

    if(!robot_state.setFromIK(
        move_group_interface_.getRobotModel()->getJointModelGroup(move_group_interface_.getName()),
        dummy_goal_pose /* this is ignored with replace = true */,
        0.0,
        moveit::core::GroupStateValidityCallbackFn(),
        ik_options))
    {
        throw IKFailedException("IK for approach pose failed");
    }

    // Plan to the goal state (but in joint space)
    move_group_interface_.setJointValueTarget(robot_state);

    // Plan
    if(!move_group_interface_.plan(plan))
    {
        throw PlanFailedException("Approach plan failed");
    };

    return plan;
}


/**
 * @brief Hit a sequence of notes in cartesian space
 *
 * @param start_state
 * @param points
 * @return moveit::planning_interface::MoveGroupInterface::Plan
**/
moveit::planning_interface::MoveGroupInterface::Plan Planning::move_to_key_points(
    const moveit_msgs::RobotState& start_state,
    std::vector<DoubleMalletKeyframe> key_frames)
{
    // Assert that there is at least one hit_point with an nice error message
    assert(key_frames.size() > 0 && "There must be at least one key frame");

    // Calculate hit trajectory
    moveit::core::RobotState robot_state(move_group_interface_.getRobotModel());
    robot_state.setToDefaultValues();
    moveit::core::robotStateMsgToRobotState(start_state, robot_state);

    // Next keyframe
    auto next_keyframe = key_frames[0];

    // Calculate the joint space plan to the mallet keyframe
    // Try the calculation 5 times if an ik or plan failed exception is thrown
    moveit::planning_interface::MoveGroupInterface::Plan plan_to_keyframe;
    int attempts = 5;
    for (int i = 0; i < attempts; i++)
    {
        try {
            plan_to_keyframe = plan_to_mallet_position(start_state, next_keyframe);
            break;
        } catch (IKFailedException& e) {
            ROS_ERROR_STREAM("IK failed: " << e.what() << " Trying again");
            if (i == attempts - 1) {
                throw PlanFailedException("IK failed after 5 attempts '" + std::string(e.what()) + "'");
            }
        } catch (PlanFailedException& e) {
            ROS_ERROR_STREAM("Plan failed: " << e.what() << " Trying again");
            if (i == attempts - 1) {
                throw PlanFailedException("Plan failed after 5 attempts '" + std::string(e.what()) + "'");
            }
        }
    }

    // Retime the trajectory to the keyframe if necessary
    // Clamp the duration to the minimum duration of the trajectory
    double duration_clamped = std::max(
        next_keyframe.duration,
        plan_to_keyframe.trajectory_.joint_trajectory.points.back().time_from_start.toSec());
    
    // Show warning if the approach time was clamped
    if(duration_clamped != next_keyframe.duration)
    {
        ROS_WARN("The keyframe approach time was clamped from %f to %f", 
            next_keyframe.duration,
            plan_to_keyframe.trajectory_.joint_trajectory.points.back().time_from_start.toSec());
    }

    auto retimed_plan = plan_to_keyframe; // slow_down_plan(plan_to_keyframe, duration_clamped);

    // Publish the plan for rviz
    moveit_msgs::DisplayTrajectory display_trajectory;
    moveit_msgs::RobotTrajectory trajectory;
    trajectory.joint_trajectory = retimed_plan.trajectory_.joint_trajectory;
    display_trajectory.trajectory_start = retimed_plan.start_state_;
    display_trajectory.trajectory.push_back(trajectory);
    trajectory_publisher_.publish(display_trajectory);

    ROS_WARN_STREAM("Number of keyframes: " << key_frames.size());

    // Wait for user input to continue
    std::cout << "Press enter to continue" << std::endl;
    
    // Wait for user input
    std::cin.ignore();

    std::cout << "Continuing" << std::endl;

    // Call hit_points recursively for all remaining hit_points
    if(key_frames.size() > 1)
    {
        auto remaining_plan = move_to_key_points(
            get_robot_state_after_plan(retimed_plan),
            std::vector<DoubleMalletKeyframe>(key_frames.begin() + 1, key_frames.end())
            );
        retimed_plan = concatinated_plan({retimed_plan, remaining_plan});
    }

    return retimed_plan;
}


/**
 * Callback for the action server
 * @param goal
 * @param action_server
 */
void Planning::action_server_callback(const marimbabot_msgs::HitSequenceGoalConstPtr &goal)
{
    try {
        // Set the max velocity and acceleration scaling factors
        move_group_interface_.setMaxVelocityScalingFactor(0.5);
        move_group_interface_.setMaxAccelerationScalingFactor(0.5);

        auto current_state = move_group_interface_.getCurrentState();
        //convert to moveit message
        moveit_msgs::RobotState start_state;
        moveit::core::robotStateToRobotStateMsg(*current_state, start_state);

        // Gets the hit points in cartesian space for every note
        auto hits = hit_sequence_to_points(
            goal->hit_sequence_elements,
            move_group_interface_.getPlanningFrame(),
            tf_buffer_
        );

        // Generate a cartesian positions for both mallets at each keypoint (approach, hit and retreat point)
        auto key_points = generate_double_trajectory(hits);

        // Define hit plan
        auto hit_plan = move_to_key_points(start_state, key_points);

        // Publish the plan for rviz
        moveit_msgs::DisplayTrajectory display_trajectory;
        moveit_msgs::RobotTrajectory trajectory;
        trajectory.joint_trajectory = hit_plan.trajectory_.joint_trajectory;
        display_trajectory.trajectory_start = hit_plan.start_state_;
        display_trajectory.trajectory.push_back(trajectory);
        trajectory_publisher_.publish(display_trajectory);

        // Execute the plan
        auto status = move_group_interface_.execute(hit_plan);
       
        // Set the result of the action server
        if (status != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            ROS_ERROR_STREAM("Hit sequence execution failed: " << status);
            marimbabot_msgs::HitSequenceResult result;
            result.success = false;
            result.error_code = marimbabot_msgs::HitSequenceResult::EXECUTION_FAILED;
            action_server_.setAborted(result);
        } else {
            marimbabot_msgs::HitSequenceResult result;
            result.success = true;
            action_server_.setSucceeded(result);
        }

    } catch (PlanFailedException& e) {
        ROS_ERROR_STREAM("Hit sequence planning failed: " << e.what());
        marimbabot_msgs::HitSequenceResult result;
        result.success = false;
        result.error_code = marimbabot_msgs::HitSequenceResult::PLANNING_FAILED;
        action_server_.setAborted(result);
    }

    // Reset last action time so we move back to the home position after a while
    last_action_time_ = ros::Time::now();
}

} // namespace marimbabot_planning

int main(int argc, char **argv)
{
    ros::init(argc, argv, "marimba_move");
    ros::AsyncSpinner spinner(4);
    spinner.start();

    marimbabot_planning::Planning planning{"arm_mallets"};

    return 0;
}