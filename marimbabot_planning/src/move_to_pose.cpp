#include <iostream>
#include "std_msgs/String.h"
#include "ros/ros.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/CollisionObject.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "marimba_move");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "marimba_arm";
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

    const moveit::core::JointModelGroup* joint_model_group =
        move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


    geometry_msgs::Pose target_pose;
    target_pose.position.x = 0.3;
    target_pose.position.y = 0.2;
    target_pose.position.z = 0.4;
    target_pose.orientation.y = sqrt(2)/2;
    target_pose.orientation.w = sqrt(2)/2;
    move_group.setPoseTarget(target_pose);
    ROS_INFO_NAMED("move_to_pose", "Setting the target position to x=%g, y=%g, z=%g",target_pose.position.x, target_pose.position.y, target_pose.position.z);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    while (true) {
        move_group_interface.setNamedTarget("home");
        move_group_interface.move();
        //move_group_interface.setNamedTarget("pose1");
        //move_group_interface.move();
        }
    return 0;
}
