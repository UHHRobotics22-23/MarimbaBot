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
    move_group_interface_.setPlanningPipelineId("ompl");
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
    geometry_msgs::PointStamped goal_point,
    geometry_msgs::PointStamped goal_point2)
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
        goal_point.point.z);


    // Copy goal point geometry_msgs::PointStamped to tf2::Vector3
    tf2::Vector3 goal_position2(
        goal_point2.point.x,
        goal_point2.point.y,
        goal_point2.point.z);

    // Use bio_ik to solve the inverse kinematics at the goal point
    bio_ik::BioIKKinematicsQueryOptions ik_options;
    ik_options.replace = true;
    ik_options.return_approximate_solution = false; // Activate for debugging if you get an error 

    // Case that only a single point / note is hit
    tf2::Quaternion goal_orientation;
    goal_orientation.setRPY(0.0, 0.0, -M_PI/4); // Rotate the mallet by 45 degrees, so that head_1 should be pointed downwards?
    ik_options.goals.emplace_back(new bio_ik::PoseGoal("mallet_head_1", goal_position, goal_orientation));

    // ik_options.goals.emplace_back(new bio_ik::PositionGoal("mallet_head_1", goal_position));
    // ik_options.goals.emplace_back(new bio_ik::PositionGoal("mallet_head_2", goal_position2));

    // Create link on plane constraint using the LinkFunctionGoal
    tf2::Vector3 plane_point(0.0, 0.0, 1.0);

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
    CartesianHitSequenceElement note1,
    CartesianHitSequenceElement note2)
{

    moveit::core::RobotState robot_state(move_group_interface_.getRobotModel());
    robot_state.setToDefaultValues();
    moveit::core::robotStateMsgToRobotState(start_state, robot_state);

    // Calculate approach point
    geometry_msgs::PointStamped approach_point1{note1.point};
    geometry_msgs::PointStamped approach_point2{note2.point};
    approach_point1.point.z += 0.1;
        
    // Calculate retreat point
    geometry_msgs::PointStamped retreat_point1{approach_point1};
    geometry_msgs::PointStamped retreat_point2{approach_point2};


    // Calculate approach trajectory
    auto approach_plan = plan_to_mallet_position(start_state, approach_point1, approach_point2);

    // Calculate down trajectory
    auto down_plan = plan_to_mallet_position(get_robot_state_after_plan(approach_plan), note1.point, note2.point);

    // Calculate retreat trajectory
    auto retreat_plan = plan_to_mallet_position(get_robot_state_after_plan(down_plan), retreat_point1, retreat_point2);

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
    moveit_msgs::RobotState& start_state,
    const std::vector<CartesianHitSequenceElement>& points1,
    const std::vector<CartesianHitSequenceElement>& points2)
{
    // Assert that there is at least one hit_point with a nice error message
    assert(points1.size() > 0 && "There must be at least one hit_point for points1");
    assert(points2.size() > 0 && "There must be at least one hit_point for points2");

    // Calculate the number of points to iterate over (take the smaller size)
    size_t num_points = std::min(points1.size(), points2.size());

    // Create an empty plan
    moveit::planning_interface::MoveGroupInterface::Plan hit_plan;

    // Iterate over the points and generate a combined plan
    for (size_t i = 0; i < num_points; ++i)
    {
        // Calculate the hit trajectory for each point in points1 and points2
        auto remaining_hit_plan = hit_note(start_state, points1[i], points2[i]);

        // Update the start state for the next iteration
        start_state = get_robot_state_after_plan(remaining_hit_plan);
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
        move_group_interface_.setMaxVelocityScalingFactor(0.9);
        move_group_interface_.setMaxAccelerationScalingFactor(0.9);

        auto current_state = move_group_interface_.getCurrentState();
        //convert to moveit message
        moveit_msgs::RobotState start_state;
        moveit::core::robotStateToRobotStateMsg(*current_state, start_state);

        // Gets the hit points in cartesian space for every note
        auto points = hit_sequence_to_points(
            goal->hit_sequence_elements, 
            move_group_interface_.getPlanningFrame(),
            tf_buffer_
        );
        auto mallet1_points = points.first;
        auto mallet2_points = points.second;
        ROS_ERROR_STREAM("Points first: " << mallet1_points.size());
        ROS_ERROR_STREAM("Points second: " << mallet2_points.size());


        // Define hit plan
        auto hit_plan = hit_notes(start_state, mallet1_points, mallet2_points);

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
