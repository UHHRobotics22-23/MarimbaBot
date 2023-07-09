#include <actionlib/server/simple_action_server.h>
#include <bio_ik/bio_ik.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <marimbabot_msgs/HitSequenceAction.h>
#include <marimbabot_planning/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <list>

namespace marimbabot_planning
{

class Planning
{
    private:
        // Initialize node handle and tf 
        ros::NodeHandle nh_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
        tf2_ros::TransformListener tf_listener_;

        // MoveIt! MoveGroupInterface
        moveit::planning_interface::MoveGroupInterface move_group_interface_;

        // Describes when a "hit" was performed in terms of 
        // how long after the start of the trajectory this point should be reached 
        std::list<ros::Duration> hit_plan_durations;

        // Action server
        actionlib::SimpleActionServer<marimbabot_msgs::HitSequenceAction> action_server_;

        // Create a trajectory publisher
        ros::Publisher trajectory_publisher_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

        // Last action time
        ros::Time last_action_time_;

        /**
         * @brief Move the robot to its idle/home position
         * 
        */
        void go_to_home_position();

        /**
         * @brief Calculate a plan so that the mallet (end effector) is at a given point in cartesian space
         *
         * @param  start_state
         * @param  goal_point
         * @return moveit::planning_interface::MoveGroupInterface::Plan
         * @throws PlanFailedException, IKFailedException
         **/
        moveit::planning_interface::MoveGroupInterface::Plan plan_to_mallet_position(
            const moveit_msgs::RobotState& start_state,
            geometry_msgs::PointStamped goal_point);


        /**
         * @brief hit a given note in cartesian space
         *
         * @param start_state
         * @param note
         * @return moveit::planning_interface::MoveGroupInterface::Plan
        **/

        moveit::planning_interface::MoveGroupInterface::Plan hit_note(
            const moveit_msgs::RobotState& start_state,
            CartesianHitSequenceElement note);


        /**
         * @brief Hit a sequence of notes in cartesian space
         *
         * @param start_state
         * @param notes
         * @return moveit::planning_interface::MoveGroupInterface::Plan
        **/
        moveit::planning_interface::MoveGroupInterface::Plan hit_notes(
            const moveit_msgs::RobotState& start_state,
            std::vector<CartesianHitSequenceElement> notes);


        /**
         * @brief Callback for the action server
         * 
         * @param goal
         * @param action_server
         */
        void action_server_callback(const marimbabot_msgs::HitSequenceGoalConstPtr &goal);

    public:
        Planning(const std::string planning_group);
};
} 
