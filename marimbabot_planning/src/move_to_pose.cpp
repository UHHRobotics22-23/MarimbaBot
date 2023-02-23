#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit/move_group_interface/move_group_interface.h>
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
 * @brief hit a given point in cartesian space
 * 
 * @param start_state
 * @param move_group_interface
 * @param pose
 * @param hit_rotation
 * @return moveit::planning_interface::MoveGroupInterface::Plan
**/
moveit::planning_interface::MoveGroupInterface::Plan hit_point(
    moveit::planning_interface::MoveGroupInterface& move_group_interface, 
    const moveit_msgs::RobotState& start_state,
    geometry_msgs::PoseStamped pose,
    double hit_rotation)
{
    // Calculate approach pose
    geometry_msgs::PoseStamped approach_pose{pose};
  
    // Convert euler angles to quaternion and rotate original quaternion
    tf2::Quaternion q_tf, q_inp;
    tf2::convert(approach_pose.pose.orientation, q_inp);
    q_tf.setRPY(0, 0, hit_rotation);
    q_tf = q_inp * q_tf;
    q_tf.normalize();
    tf2::convert(q_tf, approach_pose.pose.orientation);

    // Calculate retreat pose
    geometry_msgs::PoseStamped retreat_pose{approach_pose};

    // Calculate approach trajectory
    moveit::planning_interface::MoveGroupInterface::Plan approach_plan;
    move_group_interface.setStartState(start_state);
    move_group_interface.setPoseTarget(approach_pose);
    move_group_interface.plan(approach_plan);

    // Calculate down trajectory
    moveit::planning_interface::MoveGroupInterface::Plan down_plan;
    auto state_after_plan = get_robot_state_after_plan(approach_plan);
    move_group_interface.setStartState(state_after_plan);
    move_group_interface.setPoseTarget(pose);
    move_group_interface.plan(down_plan);

    // Calculate retreat trajectory
    moveit::planning_interface::MoveGroupInterface::Plan retreat_plan;
    state_after_plan = get_robot_state_after_plan(down_plan);
    move_group_interface.setStartState(state_after_plan);
    move_group_interface.setPoseTarget(retreat_pose);
    move_group_interface.plan(retreat_plan);

    // Concatinate trajectories
    moveit::planning_interface::MoveGroupInterface::Plan plan = concatinated_plan({approach_plan, down_plan, retreat_plan});
    return plan;
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

    const moveit::core::JointModelGroup* joint_model_group =
        move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    sensor_msgs::JointState home_joints;
    home_joints.name = { "joint_1","joint_2","joint_3","joint_4","joint_5","joint_6","joint_7" };
    home_joints.position = {-1.0764353166380913, 0.5625393633338827, 0.31906783564462415, 2.316970072925777, 0.042747545531845337, 1.0091591209316584, 1.4176498139743448};
    
    moveit::planning_interface::MoveGroupInterface::Plan plan_anywhere_to_home;
    move_group_interface.setJointValueTarget(home_joints);
    move_group_interface.plan(plan_anywhere_to_home);


    // Lookup the world to tcp transform
    geometry_msgs::TransformStamped transformStamped;
    try
    {
        transformStamped = tfBuffer.lookupTransform("base_link", "diana_gripper/tcp", ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s",ex.what());
    }

    // Convert transform to pose
    geometry_msgs::PoseStamped pose;

    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "base_link";
    pose.pose.position.x = transformStamped.transform.translation.x;
    pose.pose.position.y = transformStamped.transform.translation.y;
    pose.pose.position.z = transformStamped.transform.translation.z;
    pose.pose.orientation.x = transformStamped.transform.rotation.x;
    pose.pose.orientation.y = transformStamped.transform.rotation.y;
    pose.pose.orientation.z = transformStamped.transform.rotation.z;
    pose.pose.orientation.w = transformStamped.transform.rotation.w;

    // Print pose
    ROS_INFO_STREAM("Pose: " << pose);

    // Hit the point
    auto hit_plan = hit_point(
        move_group_interface,
        get_robot_state_after_plan(plan_anywhere_to_home),
        pose,
        0.1);

    // Print hit plan
    
    auto plan = concatinated_plan({plan_anywhere_to_home, hit_plan});

    // Publish the plan for rviz
    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;
    moveit_msgs::RobotTrajectory trajectory;
    trajectory.joint_trajectory = plan.trajectory_.joint_trajectory;
    display_trajectory.trajectory_start = plan.start_state_;
    display_trajectory.trajectory.push_back(trajectory);
    display_publisher.publish(display_trajectory);

    // Execute the plan
    move_group_interface.execute(plan);
    
    return 0;
}
