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
    geometry_msgs::PointStamped goal_point)
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
    assert(goal_point.header.frame_id == move_group_interface_.getPlanningFrame());

    // Copy goal point geometry_msgs::PointStamped to tf2::Vector3
    tf2::Vector3 goal_position(
        goal_point.point.x,
        goal_point.point.y,
        goal_point.point.z + 0.1);

    // Use bio_ik to solve the inverse kinematics at the goal point
    bio_ik::BioIKKinematicsQueryOptions ik_options;
    ik_options.replace = true;
    ik_options.return_approximate_solution = true; // Activate for debugging if you get an error 

    ik_options.goals.emplace_back(new bio_ik::PositionGoal("mallet_head_1", goal_position));

    // Create link on plane constraint using the LinkFunctionGoal
    tf2::Vector3 plane_point(0.0, 0.0, 1.3);

    // Define lambda function for link on plane constraint
    // Requested format const std::function<double(const tf2::Vector3&, const tf2::Quaternion&)>& f
    auto link_on_plane_constraint = [plane_point](const tf2::Vector3& position, const tf2::Quaternion& orientation) -> double
    {
        tf2::Vector3 plane_normal(0.0, 0.0, 1.0);
        tf2::Vector3 plane_to_position = position - plane_point;
        double signed_dist = plane_to_position.dot(plane_normal);
        // Take the squared value of the signed distance
        return std::pow(signed_dist, 2);
    };

    // Add link on plane constraint to ik_options
    ik_options.goals.emplace_back(new bio_ik::LinkFunctionGoal("ur5_wrist_1_link", link_on_plane_constraint));

    // @TODO : add quaternion constraints similar to plane to keep wrist_2_link in specific orientation
    auto orientation_constraint = [](const tf2::Vector3& position, const tf2::Quaternion& orientation) -> double
    {
        tf2::Quaternion desired_orientation;
        desired_orientation.setRPY(0.52, 0.0, 0.0); // Set roll, pitch, and yaw angles. Set Roll to 0.52 rad (30 deg)

        // Calculate the angular distance between the current and desired orientations
        tf2::Quaternion orientation_error = desired_orientation.inverse() * orientation;
        double angular_distance = 1.0 - orientation_error.dot(orientation_error);
        return std::pow(angular_distance, 2);
    };

    ik_options.goals.emplace_back(new bio_ik::LinkFunctionGoal("ur5_wrist_2_link", orientation_constraint));

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
 * @brief hit a given note in cartesian space
 *
 * @param start_state
 * @param note
 * @return moveit::planning_interface::MoveGroupInterface::Plan
**/

moveit::planning_interface::MoveGroupInterface::Plan Planning::hit_note(
    const moveit_msgs::RobotState& start_state,
    CartesianHitSequenceElement note)
{

    moveit::core::RobotState robot_state(move_group_interface_.getRobotModel());
    robot_state.setToDefaultValues();
    moveit::core::robotStateMsgToRobotState(start_state, robot_state);

    // Calculate approach point
    geometry_msgs::PointStamped approach_point{note.point};
    approach_point.point.z += 0.1;
        
    // Calculate retreat point
    geometry_msgs::PointStamped retreat_point{approach_point};

    // Calculate approach trajectory
    auto approach_plan = plan_to_mallet_position(start_state, approach_point);

    // Calculate down trajectory
    auto down_plan = plan_to_mallet_position(get_robot_state_after_plan(approach_plan), note.point);

    // Calculate retreat trajectory
    auto retreat_plan = plan_to_mallet_position(get_robot_state_after_plan(down_plan), retreat_point);


    // Set timing parameters
    std::string tone_name = note.msg.tone_name;
    int32_t octave = note.msg.octave;
    ros::Time start_time = note.msg.start_time;
    ros::Duration tone_duration = note.msg.tone_duration;
    double loudness = note.msg.loudness;

    ROS_INFO("Received data : (%s, %d, %f, %f, %f)", tone_name.c_str(), octave, loudness , start_time.toSec(), tone_duration.toSec());
    // constant scale factor retreat plan to 0.5
    retreat_plan = slow_down_plan(retreat_plan, 0.5, tone_name);
    down_plan = slow_down_plan(down_plan, loudness, tone_name);
    double timing = start_time.toSec() - down_plan.trajectory_.joint_trajectory.points.back().time_from_start.toSec() - retreat_plan.trajectory_.joint_trajectory.points.back().time_from_start.toSec();
    ROS_INFO("time calculation : %f", abs(timing));
    approach_plan = slow_down_plan(approach_plan, abs(timing), tone_name);
    
    // Concatinate trajectories
    auto plan = concatinated_plan({approach_plan, down_plan, retreat_plan});

    return plan;
}

/**
 * @brief Hit a sequence of notes in cartesian space
 *
 * @param start_state
 * @param points
 * @return moveit::planning_interface::MoveGroupInterface::Plan
**/
moveit::planning_interface::MoveGroupInterface::Plan Planning::hit_notes(
    const moveit_msgs::RobotState& start_state,
    std::vector<CartesianHitSequenceElement> points)
{
    // Assert that there is at least one hit_point with an nice error message
    assert(points.size() > 0 && "There must be at least one hit_point");

    // Calculate hit trajectory
    auto hit_plan = hit_note(
        start_state,
        points.front()
        );

    // Call hit_points recursively for all remaining hit_points
    if(points.size() > 1)
    {
        auto remaining_hit_plan = hit_notes(
            get_robot_state_after_plan(hit_plan),
            std::vector<CartesianHitSequenceElement>(points.begin() + 1, points.end())
            );
        hit_plan = concatinated_plan({hit_plan, remaining_hit_plan});
    }

    return hit_plan;
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

        // Define hit plan
        auto hit_plan = hit_notes(start_state, hits);

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
