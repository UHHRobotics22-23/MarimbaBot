#include <ros/ros.h>
#include <Eigen/Dense>
#include <string>
#include <vector>

namespace marimbabot_planning
{

enum class Mallet {
    LEFT = 0,
    RIGHT = 1
};

std::vector<Eigen::Vector4f> generate_base_trajectory(const std::vector<Eigen::Vector4f> &hit_points, float down_stroke_speed, float retreat_speed, float travel_height);
std::map<Mallet, std::vector<Eigen::Vector4f>> generate_double_trajectory(const std::vector<Eigen::Vector4f> &hit_points, float down_stroke_speed = 2.0, float retreat_speed = 2.0, float travel_height = 0.2, float time_resolution = 0.01, float max_distance = 0.2);
std::vector<Mallet> assign_mallets(const std::vector<Eigen::Vector4f> &notes);
std::map<Mallet, std::vector<Eigen::Vector4f>> split_notes_based_on_mallet(const std::vector<Eigen::Vector4f> &notes, const std::vector<Mallet> &mallet_assignment);
}
