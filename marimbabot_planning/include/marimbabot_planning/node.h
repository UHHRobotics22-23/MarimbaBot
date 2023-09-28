#pragma once

#include <actionlib/server/simple_action_server.h>
#include <dynamic_reconfigure/server.h>
#include <marimbabot_msgs/HitSequenceAction.h>
#include <marimbabot_planning/planning_paramsConfig.h>
#include <marimbabot_planning/trajectory_generation.h>
#include <marimbabot_planning/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

namespace marimbabot_planning
{

class PlanningNode
{
    private:
        // Initialize node handle and tf
        ros::NodeHandle nh_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
        tf2_ros::TransformListener tf_listener_;

        // Dynamic reconfigure server
        dynamic_reconfigure::Server<planning_paramsConfig> dyn_reconf_server_;

        // Config parameters
        planning_paramsConfig config_;

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

        /**
         * @brief Callback for the dynamic reconfigure server
         *
         * @param config
         * @param level
         */
        void dyn_reconf_callback(planning_paramsConfig &config, uint32_t level);

    public:
        PlanningNode();
};
}
