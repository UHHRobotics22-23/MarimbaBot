#include "marimbabot_planning/planning.h"

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
 * @brief Calculate a plan so that the mallet (end effector) is at a given point in cartesian space
 *
 * @param  start_state
 * @param  goal_point
 * @param  move_group_interface
 * @return moveit::planning_interface::MoveGroupInterface::Plan
 * @throws std::runtime_error
 **/
moveit::planning_interface::MoveGroupInterface::Plan plan_to_mallet_position(
    moveit::planning_interface::MoveGroupInterface& move_group_interface,
    const moveit_msgs::RobotState& start_state,
    geometry_msgs::PointStamped goal_point)
{
    // Initialize output plan
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    // Create robot state
    moveit::core::RobotState robot_state(move_group_interface.getRobotModel());
    robot_state.setToDefaultValues();
    robot_state.setVariablePositions(start_state.joint_state.name, start_state.joint_state.position);

    // Set start state
    move_group_interface.setStartState(start_state);

    // Check if goal pose is in the same frame as the planning frame
    assert(goal_point.header.frame_id == move_group_interface.getPlanningFrame());

    // Copy goal pose geometry_msgs::PointStamped to tf2::Vector3
    tf2::Vector3 goal_position(
        goal_point.point.x,
        goal_point.point.y,
        goal_point.point.z);

    // Use bio_ik to solve the inverse kinematics at the goal point
    bio_ik::BioIKKinematicsQueryOptions ik_options;
    ik_options.replace = true;
    ik_options.return_approximate_solution = false;

    ik_options.goals.emplace_back(new bio_ik::PositionGoal("mallet_head", goal_position));

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
        move_group_interface.getRobotModel()->getJointModelGroup(move_group_interface.getName()),
        dummy_goal_pose /* this is ignored with replace = true */,
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
 * @param point
 * @return moveit::planning_interface::MoveGroupInterface::Plan
**/

moveit::planning_interface::MoveGroupInterface::Plan hit_point(
    moveit::planning_interface::MoveGroupInterface& move_group_interface,
    const moveit_msgs::RobotState& start_state,
    geometry_msgs::PointStamped point)
{

    moveit::core::RobotState robot_state(move_group_interface.getRobotModel());
    robot_state.setToDefaultValues();
    moveit::core::robotStateMsgToRobotState(start_state, robot_state);

    // Calculate approach point
    geometry_msgs::PointStamped approach_point{point};
    approach_point.point.z += 0.1;
        
    // Calculate retreat point
    geometry_msgs::PointStamped retreat_point{approach_point};

    // Calculate approach trajectory
    auto approach_plan = plan_to_mallet_position(move_group_interface, start_state, approach_point);

    // Calculate down trajectory
    auto down_plan = plan_to_mallet_position(move_group_interface, get_robot_state_after_plan(approach_plan), point);

    // Calculate retreat trajectory
    auto retreat_plan = plan_to_mallet_position(move_group_interface, get_robot_state_after_plan(down_plan), retreat_point);

    // Concatinate trajectories
    auto plan = concatinated_plan({approach_plan, down_plan, retreat_plan});

    return plan;
}

/**
 * @brief Hit a sequence of points in cartesian space
 *
 * @param move_group_interface
 * @param start_state
 * @param points
 * @return moveit::planning_interface::MoveGroupInterface::Plan
**/
moveit::planning_interface::MoveGroupInterface::Plan hit_points(
    moveit::planning_interface::MoveGroupInterface& move_group_interface,
    const moveit_msgs::RobotState& start_state,
    std::vector<geometry_msgs::PointStamped> points)
{
    // Assert that there is at least one hit_point with an nice error message
    assert(points.size() > 0 && "There must be at least one hit_point");

    // Calculate hit trajectory
    auto hit_plan = hit_point(
        move_group_interface,
        start_state,
        points.front()
        );

    // Call hit_points recursively for all remaining hit_points
    if(points.size() > 1)
    {
        auto remaining_hit_plan = hit_points(
            move_group_interface,
            get_robot_state_after_plan(hit_plan),
            std::vector<geometry_msgs::PointStamped>(points.begin() + 1, points.end())
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

} // namespace marimbabot_planning

int main(int argc, char **argv)
{
    ros::init(argc, argv, "marimba_move");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Create tf2 listener
    std::shared_ptr<tf2_ros::Buffer> tfBuffer = std::make_shared<tf2_ros::Buffer>();
    tf2_ros::TransformListener tfListener(*tfBuffer);

    static const std::string PLANNING_GROUP = "arm";
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
    move_group_interface.setPlanningPipelineId("pilz_industrial_motion_planner");
    move_group_interface.setPlannerId("PTP");
    move_group_interface.setMaxVelocityScalingFactor(0.05);
    move_group_interface.setMaxAccelerationScalingFactor(0.05);
    move_group_interface.startStateMonitor();

    const moveit::core::JointModelGroup* joint_model_group =
        move_group_interface.getRobotModel()->getJointModelGroup(PLANNING_GROUP);

    move_group_interface.setNamedTarget("marimbabot_home");
    move_group_interface.move();

    move_group_interface.setMaxVelocityScalingFactor(0.9);
    move_group_interface.setMaxAccelerationScalingFactor(0.9);
    //ros::Duration(2).sleep();

    // Lookup the world to tcp transform
    geometry_msgs::TransformStamped transformStamped;
    try
    {
        transformStamped = tfBuffer.lookupTransform("world", "ft_fts_toolside", ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s",ex.what());
    }

    // Convert transform to pose
    geometry_msgs::PointStamped point;

    point.header.stamp = ros::Time::now();
    point.header.frame_id = transformStamped.header.frame_id;
    point.point.x = transformStamped.transform.translation.x;
    point.point.y = transformStamped.transform.translation.y;
    point.point.z = transformStamped.transform.translation.z;

    // Print pose
    ROS_INFO_STREAM("Pose: " << point);

    auto current_state = move_group_interface.getCurrentState();
    //convert to moveit message
    moveit_msgs::RobotState start_state;
    moveit::core::robotStateToRobotStateMsg(*current_state, start_state);

    // Hit a sequence of points in cartesian space left and right of "pose"

    // Define hit points
    std::vector<geometry_msgs::PointStamped> hit_point1;
    for (auto offset : {-0.1, -0.05, 0.0, 0.05, 0.1})
    {
        geometry_msgs::PointStamped hit_point{point};
        hit_point.point.y += offset;
        hit_point1.push_back(hit_point);
        ROS_INFO_STREAM("Pose point : " << hit_point);
    }

    // Define hit plan by mapping hit_point on hit_points
    auto hit_plan = hit_points(move_group_interface, start_state, hit_point1);

    // Publish the plan for rviz
    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;
    moveit_msgs::RobotTrajectory trajectory;
    trajectory.joint_trajectory = hit_plan.trajectory_.joint_trajectory;
    display_trajectory.trajectory_start = hit_plan.start_state_;
    display_trajectory.trajectory.push_back(trajectory);
    display_publisher.publish(display_trajectory);
    std::cout <<"joint tracjector "<< trajectory.joint_trajectory;
    

    // Execute the plan
    move_group_interface.execute(hit_plan);
    //move_group_interface.plan(hit_plan);
    ros::waitForShutdown();
    return 0;
}
