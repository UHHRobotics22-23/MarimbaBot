#include <iostream>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
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


int main(int argc, char **argv)
{
    ros::init(argc, argv, "marimba_move");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "arm_with_tcp";
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
    move_group_interface.setPlanningPipelineId("pilz_industrial_motion_planner");
    move_group_interface.setPlannerId("PTP");
    move_group_interface.setMaxVelocityScalingFactor(0.9);
    move_group_interface.setMaxAccelerationScalingFactor(0.9);

    const moveit::core::JointModelGroup* joint_model_group =
        move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    sensor_msgs::JointState down_joints, up_joints;
    up_joints.name = { "joint_1","joint_2","joint_3","joint_4","joint_5","joint_6","joint_7" };
    up_joints.position = {-1.0764353166380913, 0.5625393633338827, 0.31906783564462415, 2.316970072925777, 0.042747545531845337, 1.0091591209316584, 1.4176498139743448};
    
    down_joints.name = { "joint_1","joint_2","joint_3","joint_4","joint_5","joint_6","joint_7" };
    down_joints.position = {-1.076231584802557, 0.5324589570285054, 0.3292064905188665, 2.316958088700157, 0.08015031369084147, 0.6043505158061902, 1.4166191705710522};

    moveit::planning_interface::MoveGroupInterface::Plan plan_anywhere_to_up, plan_up_to_down, plan_down_to_up;
    move_group_interface.setJointValueTarget(up_joints);
    move_group_interface.plan(plan_anywhere_to_up);

    moveit_msgs::RobotState state_after_plan {plan_anywhere_to_up.start_state_};

    state_after_plan.joint_state = up_joints;
    move_group_interface.setStartState(state_after_plan);
    move_group_interface.setJointValueTarget(down_joints);
    move_group_interface.plan(plan_up_to_down);

    state_after_plan.joint_state = down_joints;
    move_group_interface.setStartState(state_after_plan);
    move_group_interface.setJointValueTarget(up_joints);
    move_group_interface.plan(plan_down_to_up);

    move_group_interface.execute(plan_anywhere_to_up);

    std::vector<moveit::planning_interface::MoveGroupInterface::Plan> plans;
    plans.push_back(plan_anywhere_to_up);

    // More strokes
    for(int i = 0; i < 10; i++)
    {
        plans.push_back(plan_up_to_down);
        plans.push_back(plan_down_to_up);
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan = concatinated_plan(plans);

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
