#include <boost/algorithm/string.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <regex>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/Marker.h>


namespace marimbabot_planning
{

/**
 * @brief Convert lilypond sequence to cartesian poses and times
 *
 * @param tf_buffer
 * @param planning_frame
 * @param lilypond
 * @param tempo
 * @return std::vector<std::tuple<geometry_msgs::PoseStamped, double, double>> (cartesian pose, start time, duration)
 **/
std::vector<std::tuple<geometry_msgs::PoseStamped, double, double>> lilypond_to_cartesian(
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::string& planning_frame,
    std::string lilypond,
    double tempo);   

}  // namespace marimbabot_planning