#pragma once

#include <actionlib/server/simple_action_server.h>
#include <marimbabot_msgs/HitSequenceAction.h>
#include <marimbabot_planning/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <marimbabot_planning/trajectory_generation.h>

namespace marimbabot_planning
{

class PlanningNode
{
    private:
        // Initialize node handle and tf
        ros::NodeHandle nh_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
        tf2_ros::TransformListener tf_listener_;

        // Define trajectory generator
        TrajectoryGenerator trajectory_generator_;

        // Action server
        actionlib::SimpleActionServer<marimbabot_msgs::HitSequenceAction> action_server_;

        // Create a trajectory publisher
        ros::Publisher trajectory_publisher_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

        // Last action time
        ros::Time last_action_time_;

        /**
         * @brief Callback for the action server
         *
         * @param goal
         * @param action_server
         */
        void action_server_callback(const marimbabot_msgs::HitSequenceGoalConstPtr &goal);

    public:
        PlanningNode();
};
}
