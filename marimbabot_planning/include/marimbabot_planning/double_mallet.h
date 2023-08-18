#include <ros/ros.h>
#include <Eigen/Dense>
#include <string>
#include <vector>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;


namespace marimbabot_planning
{

enum class Mallet {
    LEFT = 0,
    RIGHT = 1
};

std::vector<Eigen::Vector4f>  generateBaseTrajectory(const Eigen::MatrixXf& points, const Eigen::VectorXf& timings, float down_stroke_speed, float retreat_speed, float travel_height);
map<Mallet, std::vector<Eigen::Vector4f>> generateDoubleTrajectory(const Eigen::MatrixXf& mallet_0_notes, const Eigen::MatrixXf& mallet_1_notes, float down_stroke_speed = 2.0, float retreat_speed = 2.0, float travel_height = 0.2, float time_resolution = 0.01, float max_distance = 0.2);
std::vector<Mallet> assignMallets(const Eigen::MatrixXf& notes);
std::vector<std::vector<Eigen::RowVector4f>> splitNotesBasedOnMallet(const Eigen::MatrixXf& notes, const std::vector<Mallet>& mallet_assignment);
}
