#include <actionlib/server/simple_action_server.h>
#include <bio_ik/bio_ik.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <marimbabot_msgs/HitSequenceAction.h>
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


namespace marimbabot_planning
{

class Planning
{
    private:
        // Initialize node handle and tf 
        ros::NodeHandle nh_;
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        // MoveIt! MoveGroupInterface
        moveit::planning_interface::MoveGroupInterface move_group_interface_;

        // Action server
        actionlib::SimpleActionServer<marimbabot_msgs::HitSequenceAction> action_server_;

        // Create a trajectory publisher
        ros::Publisher trajectory_publisher_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

        /**
         * @brief concatinates a vector of n plans (n>0) into one plan
         *
         * @param plans
         * @return moveit::planning_interface::MoveGroupInterface::Plan
        **/
        moveit::planning_interface::MoveGroupInterface::Plan concatinated_plan(std::vector<moveit::planning_interface::MoveGroupInterface::Plan> plans);


        /**
         * @brief get robot state after plan
         *
         * @param  plan
         * @return moveit_msgs::RobotState
        **/
        moveit_msgs::RobotState get_robot_state_after_plan(moveit::planning_interface::MoveGroupInterface::Plan plan);


        /**
         * @brief Calculate a plan so that the mallet (end effector) is at a given point in cartesian space
         *
         * @param  start_state
         * @param  goal_point
         * @return moveit::planning_interface::MoveGroupInterface::Plan
         * @throws std::runtime_error
         **/
        moveit::planning_interface::MoveGroupInterface::Plan plan_to_mallet_position(
            const moveit_msgs::RobotState& start_state,
            geometry_msgs::PointStamped goal_point);


        /**
         * @brief hit a given point in cartesian space
         *
         * @param start_state
         * @param pointstart_state
         * @return moveit::planning_interface::MoveGroupInterface::Plan
        **/

        moveit::planning_interface::MoveGroupInterface::Plan hit_point(
            const moveit_msgs::RobotState& start_state,
            geometry_msgs::PointStamped point);

        /**
         * @brief Hit a sequence of points in cartesian space
         *
         * @param start_state
         * @param points
         * @return moveit::planning_interface::MoveGroupInterface::Plan
        **/
        moveit::planning_interface::MoveGroupInterface::Plan hit_points(
            const moveit_msgs::RobotState& start_state,
            std::vector<geometry_msgs::PointStamped> points);


        /**
         * @brief Slow down a trajectory to a given length. No speedup is possible.
         *
         * @param input_plan
         * @param length
         * @return moveit::planning_interface::MoveGroupInterface::Plan
         **/

        moveit::planning_interface::MoveGroupInterface::Plan slow_down_plan(
            const moveit::planning_interface::MoveGroupInterface::Plan& input_plan,
            double length);

        /**
         * Callback for the action server
         * @param goal
         * @param action_server
         */
        void action_server_callback(const marimbabot_msgs::HitSequenceGoalConstPtr &goal);

    public:
        Planning(const std::string planning_group);
};
} 
