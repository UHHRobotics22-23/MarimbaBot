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
 * @param  wrist_height
 * @return moveit::planning_interface::MoveGroupInterface::Plan
 * @throws PlanFailedException, IKFailedException
 **/
moveit::planning_interface::MoveGroupInterface::Plan Planning::plan_to_mallet_position(
    const moveit_msgs::RobotState& start_state,
    geometry_msgs::PointStamped left_mallet_goal_point,
    boost::optional<geometry_msgs::PointStamped> right_mallet_goal_point,
    double wrist_height)
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

    // Check if goal points are in the same frame as the planning frame
    assert(left_mallet_goal_point.header.frame_id == move_group_interface_.getPlanningFrame());
    if (right_mallet_goal_point)
    {
        assert(right_mallet_goal_point->header.frame_id == move_group_interface_.getPlanningFrame());
    }

    // Use bio_ik to solve the inverse kinematics at the goal point
    bio_ik::BioIKKinematicsQueryOptions ik_options;
    ik_options.replace = true;
    ik_options.return_approximate_solution = false; // Activate for debugging if you get an error 

    // Add link on plane constraint to ik_options that holds the wrist at the same height
    ik_options.goals.emplace_back(new bio_ik::LinkFunctionGoal("ur5_wrist_1_link", link_on_plane_constraint(tf2::Vector3(0.0, 0.0, wrist_height))));

    // Copy goal point geometry_msgs::PointStamped to tf2::Vector3
    tf2::Vector3 left_mallet_goal_position(
        left_mallet_goal_point.point.x,
        left_mallet_goal_point.point.y,
        left_mallet_goal_point.point.z + 0.01);  

    // Add goal to ik_options
    ik_options.goals.emplace_back(new bio_ik::PositionGoal("mallet_head_1", left_mallet_goal_position));

    // Check if we have a goal for the right mallet
    if(right_mallet_goal_point)
    {
        // Copy goal point geometry_msgs::PointStamped to tf2::Vector3
        tf2::Vector3 right_mallet_goal_position(
            right_mallet_goal_point->point.x,
            right_mallet_goal_point->point.y,
            right_mallet_goal_point->point.z + 0.01);  

        // Add goal to ik_options
        ik_options.goals.emplace_back(new bio_ik::PositionGoal("mallet_head_2", right_mallet_goal_position));
    } else {
        // Perform idle behavior for the right mallet

        // Add joint variable goal for the wrist joint to avoid unnecessary rotations
        ik_options.goals.emplace_back(new bio_ik::JointVariableGoal("ur5_wrist_3_joint", 0.0));

        // Double mallet specific goals that move the second mallet out of the way
        // Add link on plane constraint to hold the second mallet head in place
        ik_options.goals.emplace_back(new bio_ik::LinkFunctionGoal("mallet_head_2", link_on_plane_constraint(
            tf2::Vector3(0.0, 0.0, left_mallet_goal_position.z() + 0.15))));

        // Keep the double mallet joint at 70 degrees
        ik_options.goals.emplace_back(new bio_ik::JointVariableGoal("mallet_finger", 45.0 * M_PI / 180.0));
    }
    
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
    // Set robot state to the given start state
    moveit::core::RobotState robot_state(move_group_interface_.getRobotModel());
    robot_state.setToDefaultValues();
    moveit::core::robotStateMsgToRobotState(start_state, robot_state);
    
    // Calculate approach points
    geometry_msgs::PointStamped left_mallet_approach_point{note.left_mallet_point};
    left_mallet_approach_point.point.z += 0.18;
    left_mallet_approach_point.point.x += 0.09;
    boost::optional<geometry_msgs::PointStamped> right_mallet_approach_point{note.right_mallet_point};
    if(right_mallet_approach_point)
    {
        right_mallet_approach_point->point.z += 0.18;
        right_mallet_approach_point->point.x += 0.09;
    }
        
    // Calculate retreat point
    geometry_msgs::PointStamped left_mallet_retreat_point{left_mallet_approach_point};
    boost::optional<geometry_msgs::PointStamped> right_mallet_retreat_point{right_mallet_approach_point};

    // Calculate the wrist height
    double wrist_height = note.left_mallet_point.point.z + 0.34;

    // Calculate approach trajectory
    auto approach_plan = plan_to_mallet_position(start_state, left_mallet_approach_point, right_mallet_approach_point, wrist_height);

    // Calculate down trajectory
    auto down_plan = plan_to_mallet_position(get_robot_state_after_plan(approach_plan), note.left_mallet_point, note.right_mallet_point, wrist_height);

    // Calculate retreat trajectory
    auto retreat_plan = plan_to_mallet_position(get_robot_state_after_plan(down_plan), left_mallet_retreat_point, right_mallet_retreat_point, wrist_height);

    // Set timing parameters
    std::string tone_name = note.msg.tone_name;
    ros::Duration tone_duration = note.msg.tone_duration;
    int32_t octave = note.msg.octave;
    ros::Time start_time = note.msg.start_time;
    double loudness = note.msg.loudness;
    ROS_DEBUG("Received data : (%s, %d, %f, %f, %f)", tone_name.c_str(), octave, loudness , start_time.toSec(), tone_duration.toSec());

    double most_silent_hit_duration = 0.3; // in seconds
    ros::Duration down_stroke_duration(
        // Get the duration of the fastest possible down stroke
        down_plan.trajectory_.joint_trajectory.points.back().time_from_start.toSec() + \
        // Extend the duration of the down stroke by the inverse of the loudness
        (1 - loudness) * most_silent_hit_duration);
    down_plan = slow_down_plan(down_plan, down_stroke_duration.toSec());
    double approach_time = start_time.toSec() - \
        down_plan.trajectory_.joint_trajectory.points.back().time_from_start.toSec() - \
        retreat_plan.trajectory_.joint_trajectory.points.back().time_from_start.toSec();
    
    // Clamp the approach time to the minimum duration of the approach trajectory
    double approach_time_clamped = std::max(approach_time, approach_plan.trajectory_.joint_trajectory.points.back().time_from_start.toSec());
    // Show warning if the approach time was clamped
    if(approach_time_clamped != approach_time)
    {
        ROS_WARN("Approach time was clamped from %f to %f", approach_time, approach_time_clamped);
    }
    approach_plan = slow_down_plan(approach_plan, approach_time_clamped);
    
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

    std::vector<ros::Duration> note_hit_times;
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

    // initialize feedback
    marimbabot_msgs::HitSequenceFeedback feedback;
    feedback.playing = true;
    feedback.executed_sequence_elements.clear();

    try {
        // Set the max velocity and acceleration scaling factors
        move_group_interface_.setMaxVelocityScalingFactor(0.95);
        move_group_interface_.setMaxAccelerationScalingFactor(0.95);

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

        // Convert the timing information in the hits from absolute to relative
        auto hits_relative = hit_sequence_absolute_to_relative(hits);

        // Detect chords and apply them as goals for the second mallet
        auto hits_relative_with_chords = apply_chords(hits_relative);

        // Define hit plan
        auto hit_plan = hit_notes(start_state, hits_relative_with_chords);

        // Publish the plan for rviz
        moveit_msgs::DisplayTrajectory display_trajectory;
        moveit_msgs::RobotTrajectory trajectory;
        trajectory.joint_trajectory = hit_plan.trajectory_.joint_trajectory;
        display_trajectory.trajectory_start = hit_plan.start_state_;
        display_trajectory.trajectory.push_back(trajectory);
        trajectory_publisher_.publish(display_trajectory);


        // set the start time of execution
        ros::Time start_time = ros::Time::now();

        // Publish the feedback
        action_server_.publishFeedback(feedback);

        // Execute the plan
        auto status = move_group_interface_.execute(hit_plan);
        // Set playing to false
        feedback.playing = false;

        ros::Duration plan_execution_time = ros::Time::now() - start_time;

        // Set the result of the action server
        if (status != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            ROS_ERROR_STREAM("Hit sequence execution failed: " << status);
            marimbabot_msgs::HitSequenceResult result;
            result.success = false;
            result.error_code = marimbabot_msgs::HitSequenceResult::EXECUTION_FAILED;
            action_server_.setAborted(result);
        } else {
            marimbabot_msgs::HitSequenceResult result;
            result.executed_sequence_elements = feedback.executed_sequence_elements;
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