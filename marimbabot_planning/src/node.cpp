#include "marimbabot_planning/node.h"

namespace marimbabot_planning
{

/**
 * @brief Construct a new Planning:: Planning object
 *
**/
PlanningNode::PlanningNode() :
    nh_{},
    tf_listener_{*tf_buffer_},
    dyn_reconf_server_{nh_},
    trajectory_generator_{tf_buffer_},
    action_server_{
        nh_,
        "hit_sequence",
        boost::bind(&PlanningNode::action_server_callback, this, _1),
        false}
{
    // Start dynamic reconfigure server
    dyn_reconf_server_.setCallback(boost::bind(&PlanningNode::dyn_reconf_callback, this, _1, _2));
    // Move to home position
    trajectory_generator_.go_to_home_position();
    // Timer callback that moves back to the home position if no action is active
    auto timer_callback = [this](const ros::TimerEvent& event) {
        // Check if an action in not active and if the last action was more than 5 seconds ago
        if (!action_server_.isActive() && ros::Time::now() - last_action_time_ > ros::Duration(1.0))
        {
            // Check if we are currently close to the home position
            trajectory_generator_.go_to_home_position();
        }
    };
    // Create timer that checks if an action is active and moves back to the home position if not
    auto timer = nh_.createTimer(ros::Duration(0.5), timer_callback);
    // Start action server
    action_server_.start();
}


/**
 * @brief Callback for the dynamic reconfigure server
 *
 * @param config
 * @param level
 */
void PlanningNode::dyn_reconf_callback(planning_paramsConfig &config, uint32_t level)
{
    // Update config
    config_ = config;
}


/**
 * Callback for the action server
 * @param goal
 * @param action_server
 */
void PlanningNode::action_server_callback(const marimbabot_msgs::HitSequenceGoalConstPtr &goal)
{
    // Initialize feedback
    marimbabot_msgs::HitSequenceFeedback feedback;
    feedback.playing = false;
    // Publish the feedback
    action_server_.publishFeedback(feedback);

    try {
        // Generate the trajectory
        auto hit_plan = trajectory_generator_.hit_notes(goal->hit_sequence_elements);

        // Publish the plan for rviz
        moveit_msgs::DisplayTrajectory display_trajectory;
        moveit_msgs::RobotTrajectory trajectory;
        trajectory.joint_trajectory = hit_plan.trajectory_.joint_trajectory;
        display_trajectory.trajectory_start = hit_plan.start_state_;
        display_trajectory.trajectory.push_back(trajectory);
        trajectory_publisher_.publish(display_trajectory);

        // Publish the feedback
        feedback.playing = true;
        action_server_.publishFeedback(feedback);

        // Execute the plan
        trajectory_generator_.execute_plan(hit_plan);

        // Set playing to false
        feedback.playing = false;
        action_server_.publishFeedback(feedback);

    } catch (PlanFailedException& e) {
        ROS_ERROR_STREAM("Hit sequence planning failed: " << e.what() << std::endl);
        marimbabot_msgs::HitSequenceResult result;
        result.success = false;
        result.error_code = marimbabot_msgs::HitSequenceResult::PLANNING_FAILED;
        action_server_.setAborted(result);
    } catch (ExecutionFailedException& e) {
        ROS_ERROR_STREAM("Hit sequence execution failed: " << e.what() << std::endl);
        marimbabot_msgs::HitSequenceResult result;
        result.success = false;
        result.error_code = marimbabot_msgs::HitSequenceResult::EXECUTION_FAILED;
        action_server_.setAborted(result);
    }

    // Reset last action time so we move back to the home position after a while
    last_action_time_ = ros::Time::now();

    // Set the result
    marimbabot_msgs::HitSequenceResult result;
    result.success = true;
    action_server_.setSucceeded(result);
}

} // namespace marimbabot_planning

int main(int argc, char **argv)
{
    ros::init(argc, argv, "marimba_move");
    ros::AsyncSpinner spinner(4);
    spinner.start();

    marimbabot_planning::PlanningNode planning_node;

    ros::waitForShutdown();

    return 0;
}
