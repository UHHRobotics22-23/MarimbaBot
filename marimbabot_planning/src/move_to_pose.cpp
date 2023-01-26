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

    const moveit::core::JointModelGroup* joint_model_group =
        move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Duration(1).sleep();

    geometry_msgs::TransformStamped transformStamped;
    try{
        transformStamped = tfBuffer.lookupTransform("world", "diana_gripper/tcp", ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      return 1;
    }

    geometry_msgs::Pose target_pose;
    target_pose.position.x = transformStamped.transform.translation.x+0.05;
    target_pose.position.y = transformStamped.transform.translation.y+0.05;
    target_pose.position.z = transformStamped.transform.translation.z+0.05;
    target_pose.orientation.x = transformStamped.transform.rotation.x;
    target_pose.orientation.y = transformStamped.transform.rotation.y;
    target_pose.orientation.z = transformStamped.transform.rotation.z;
    target_pose.orientation.w = transformStamped.transform.rotation.w;

    move_group_interface.setPoseTarget(target_pose);

    ROS_INFO_NAMED("move_to_pose", "target position is set x=%g, y=%g, z=%g",target_pose.position.x, target_pose.position.y, target_pose.position.z);
    move_group_interface.move();
    return 0;
}
