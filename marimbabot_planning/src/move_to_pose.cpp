#include <iostream>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>



int main(int argc, char **argv)
{
    ros::init(argc, argv, "marimba_move");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "arm_with_tcp";
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
    move_group_interface.setPlanningPipelineId("ompl");
    move_group_interface.setPlannerId("ompl");
    move_group_interface.setMaxVelocityScalingFactor(0.9);
    move_group_interface.setMaxAccelerationScalingFactor(0.9);

    const moveit::core::JointModelGroup* joint_model_group =
        move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    sensor_msgs::JointState down_joints, up_joints;
    down_joints.name = { "joint_1","joint_2","joint_3","joint_4","joint_5","joint_6","joint_7" };
    down_joints.position = {-0.7426866173558437, 0.8284573456087861, -0.03405935646463343, 1.6107396571585018, 0.09181096521878239, 0.2238119037082843, 1.5289353330786188};

    up_joints.name = { "joint_1","joint_2","joint_3","joint_4","joint_5","joint_6","joint_7" };
    up_joints.position = {-0.7426866173558437, 0.8284573456087861, -0.03405935646463343, 1.7039409798026908, 0.09181096521878239, 0.890063508152813, 1.5289353330786188};

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



    return 0;
}
