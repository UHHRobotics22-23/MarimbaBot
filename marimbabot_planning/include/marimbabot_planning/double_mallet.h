#include <Eigen/Dense>
#include <geometry_msgs/PointStamped.h>
#include <marimbabot_planning/utils.h>
#include <ros/ros.h>
#include <string>
#include <vector>

namespace marimbabot_planning
{

enum class Mallet {
    LEFT = 0,
    RIGHT = 1
};

struct DoubleMalletKeyframe {
    bool left_mallet_active = false;
    geometry_msgs::PointStamped left_mallet_position;

    bool right_mallet_active = false;
    geometry_msgs::PointStamped right_mallet_position;

    double duration = 0;
};

std::vector<Eigen::Vector4f> generate_base_trajectory(const std::vector<Eigen::Vector4f> &hit_points, float travel_height);
std::vector<DoubleMalletKeyframe> generate_double_trajectory(const std::vector<CartesianHitSequenceElement> &hit_points, float travel_height = 0.2, float time_resolution = 0.01, float max_distance = 0.2);
std::vector<Mallet> assign_mallets(const std::vector<CartesianHitSequenceElement> &notes);
std::map<Mallet, std::vector<CartesianHitSequenceElement>> split_notes_based_on_mallet(const std::vector<CartesianHitSequenceElement> &notes, const std::vector<Mallet> &mallet_assignment);
}
