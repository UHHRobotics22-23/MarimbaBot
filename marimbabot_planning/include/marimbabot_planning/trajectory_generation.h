#pragma once

#include <bio_ik/bio_ik.h>
#include <boost/optional.hpp>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <marimbabot_msgs/HitSequenceElement.h>
#include <marimbabot_planning/utils.h>
#include <moveit_msgs/RobotState.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace marimbabot_planning
{

class TrajectoryGenerator
{
    private:
        // Keep a reference to the tf buffer
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

        // MoveIt! MoveGroupInterface
        moveit::planning_interface::MoveGroupInterface move_group_interface_;

    public:
        TrajectoryGenerator(std::shared_ptr<tf2_ros::Buffer> tf_buffer);

        /**
         * @brief Calculate a plan so that the mallet (end effector) is at a given point in cartesian space
         *
         * @param  start_state
         * @param  left_mallet_goal_point
         * @param  right_mallet_goal_point
         * @param  wrist_height
         * @return moveit::planning_interface::MoveGroupInterface::Plan
         * @throws PlanFailedException, IKFailedException
         **/
        moveit::planning_interface::MoveGroupInterface::Plan plan_to_mallet_position(
            const moveit_msgs::RobotState& start_state,
            geometry_msgs::PointStamped left_mallet_goal_point,
            boost::optional<geometry_msgs::PointStamped> right_mallet_goal_point = boost::none,
            double wrist_height = 1.3);

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
         * @brief Hit a sequence of notes in cartesian space with relative timing
         *
         * @param start_state
         * @param notes
         * @return moveit::planning_interface::MoveGroupInterface::Plan
        **/
        moveit::planning_interface::MoveGroupInterface::Plan hit_notes_cartesian(
            const moveit_msgs::RobotState& start_state,
            std::vector<CartesianHitSequenceElement> notes);

        /**
         * @brief Generate a plan to hit a sequence of notes
         *
         * @param hit_sequence
         * @return moveit::planning_interface::MoveGroupInterface::Plan
        **/
        moveit::planning_interface::MoveGroupInterface::Plan hit_notes(
            const std::vector<marimbabot_msgs::HitSequenceElement>& notes);

        /**
         * @brief Execute a plan
         *
         * @param plan
         * @return void
         * @throws ExecutionFailedException
         **/
        void execute_plan(const moveit::planning_interface::MoveGroupInterface::Plan& plan);

        /**
         * @brief Move the robot to its idle/home position
         *
        */
        void go_to_home_position();
};
}
