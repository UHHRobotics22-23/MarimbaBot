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
        false}
{
    // Set planning pipeline and planner
    move_group_interface_.setPlanningPipelineId("pilz_industrial_motion_planner");
    move_group_interface_.setPlannerId("PTP");
    move_group_interface_.startStateMonitor();
    // Move to home position
    go_to_home_position();
    // Timer callback that moves back to the home position if no action is active
    auto timer_callback = [this](const ros::TimerEvent& event) {
        // Check if an action in not active and if the last action was more than 5 seconds ago
        if (!action_server_.isActive() && ros::Time::now() - last_action_time_ > ros::Duration(1.0))
        {
            // Check if we are currently close to the home position
            go_to_home_position();
        }
    };
    // Create timer that checks if an action is active and moves back to the home position if not
    auto timer = nh_.createTimer(ros::Duration(0.5), timer_callback);
    // Start action server
    action_server_.start();
    // Wait for shutdown
    ros::waitForShutdown();
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

    // Create robot state
    moveit::core::RobotState robot_state(move_group_interface_.getRobotModel());
    robot_state.setToDefaultValues();
    robot_state.setVariablePositions(start_state.joint_state.name, start_state.joint_state.position);

    // Set start state
    move_group_interface_.setStartState(start_state);

    // Check if goal point is in the same frame as the planning frame
    assert(goal.left_mallet_active && (goal.left_mallet_position.header.frame_id == move_group_interface_.getPlanningFrame()));
    assert(goal.right_mallet_active && (goal.right_mallet_position.header.frame_id == move_group_interface_.getPlanningFrame()));

    // Check if none of the mallets is defined else print error
    assert(goal.left_mallet_active || goal.right_mallet_active);

    // Copy goals from geometry_msgs::PointStamped to tf2::Vector3
    tf2::Vector3 left_mallet_goal_position(
        goal.left_mallet_position.point.x,
        goal.left_mallet_position.point.y,
        goal.left_mallet_position.point.z);

    tf2::Vector3 right_mallet_goal_position(
        goal.right_mallet_position.point.x,
        goal.right_mallet_position.point.y,
        goal.right_mallet_position.point.z);

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
    if (!(goal.left_mallet_active && goal.right_mallet_active)) {
        // Keep the double mallet joint at 70 degrees
        ik_options.goals.emplace_back(new bio_ik::JointVariableGoal("mallet_finger", 60.0 * M_PI / 180.0));
        // Add joint variable goal for the wrist joint to avoid unnecessary rotations
        ik_options.goals.emplace_back(new bio_ik::JointVariableGoal("ur5_wrist_3_joint", 0.0));
    }

    // Add link on plane constraint to ik_options
    ik_options.goals.emplace_back(new bio_ik::LinkFunctionGoal("ur5_wrist_1_link", link_on_plane_constraint(tf2::Vector3(0.0, 0.0, 1.3))));

    // Create minimal displacement goal, so that the robot does not move too much and stays close to the start state
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
    auto plan_to_keyframe = plan_to_mallet_position(start_state, key_frames[0]);

    // Retime the trajectory to the keyframe if necessary
    // Clamp the duration to the minimum duration of the trajectory
    double duration_clamped = std::max(
        next_keyframe.duration,
        plan_to_keyframe.trajectory_.joint_trajectory.points.back().time_from_start.toSec());
    
    // Show warning if the approach time was clamped
    if(duration_clamped != next_keyframe.duration)
    {
        ROS_WARN("The keyframe approach time was clamped from %f to %f", duration_clamped, next_keyframe.duration);
    }

    auto retimed_plan = slow_down_plan(plan_to_keyframe, duration_clamped);

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

    marimbabot_planning::Planning planning{"arm"};

    return 0;
}
