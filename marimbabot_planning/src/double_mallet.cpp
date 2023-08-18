#include "marimbabot_planning/double_mallet.h"

namespace marimbabot_planning
{

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <iterator>

#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

vector<Vector4f> generate_base_trajectory(const vector<Vector4f>& hit_points, float down_stroke_speed, float retreat_speed, float travel_height) {
    vector<Vector4f> result;

    for (int i = 0; i < hit_points.size(); ++i) {
        // Create the approach point
        result.push_back(points[i] + Vector4f(0.0, 0.0, travel_height, -travel_height / down_stroke_speed));
        // Copy over the hit point
        result.push_back(points[i]);
        // Create the retreat point
        result.push_back(points[i] + Vector4f(0.0, 0.0, travel_height, travel_height / retreat_speed));
    }

    return result;
}

map<Mallet, vector<Vector4f>> generate_double_trajectory(const vector<Vector4f>& hit_points, float down_stroke_speed = 2.0, float retreat_speed = 2.0, float travel_height = 0.2, float time_resolution = 0.01, float max_distance = 0.2) {

    // Assign each hit point a mallet
    vector<Mallet> mallet_assignment = assign_mallets(hit_points);

    // Split the hit points into two vectors based on the mallet assignment
    vector<vector<Vector4f>> split_hit_points = split_notes_based_on_mallet(hit_points, mallet_assignment);


    // Create a mapping from mallet (enum) to base trajectory
    map<Mallet, vector<Vector4f>> mallet_base_trajectories = {
        {Mallet::LEFT, generate_base_trajectory(mallet_0_notes, down_stroke_speed, retreat_speed, travel_height)},
        {Mallet::RIGHT, generate_base_trajectory(mallet_1_notes, down_stroke_speed, retreat_speed, travel_height)}};


    int maximum_number_of_timestamps = max(
        mallet_base_trajectories[Mallet::LEFT].size(),
        mallet_base_trajectories[Mallet::RIGHT].size());
        
    for (int i = 0; i < maximum_number_of_timestamps; ++i) {

        // Check if we've reached the end of one of the trajectories (no more double planning is needed for the rest)
        if (i >= mallet_base_trajectories[Mallet::LEFT].size() && i >= mallet_base_trajectories[Mallet::RIGHT].size()) {
            break; // TODO handle this better
        }

        float left_mallet_timestamp = mallet_base_trajectories[Mallet::LEFT][i](3);
        float right_mallet_timestamp = mallet_base_trajectories[Mallet::RIGHT][i](3);

        Mallet current_mallet, other_mallet_number;

        if (left_mallet_timestamp < right_mallet_timestamp) {
            current_mallet = Mallet::LEFT;
            other_mallet_number = Mallet::RIGHT;
        } else if (left_mallet_timestamp > right_mallet_timestamp) {
            current_mallet = Mallet::RIGHT;
            other_mallet_number = Mallet::LEFT;
        } else {
            // We have a cord strike, so we can just skip this timestamp
            continue;
        }

        Vector4f current_mallet_state = mallet_base_trajectories[current_mallet][i];
        Vector4f other_mallet_previous_state = mallet_base_trajectories[other_mallet_number][max(i - 1, 0)];
        Vector4f other_mallet_next_state = mallet_base_trajectories[other_mallet_number][i];

        Vector4f other_mallet_current_state;

        // Check if the other mallet previous state and future state are the same
        if (other_mallet_previous_state(3) == other_mallet_next_state(3)) {
            // Copy over the previous state
            other_mallet_current_state = other_mallet_previous_state;

        } else {
            // We can interpolate the other mallet's state
            other_mallet_current_state = other_mallet_previous_state + (other_mallet_next_state - other_mallet_previous_state) * (current_mallet_state(3) - other_mallet_previous_state(3)) / (other_mallet_next_state(3) - other_mallet_previous_state(3));
        }

        float distance = (current_mallet_state.head(3) - other_mallet_current_state.head(3)).norm();

        if (distance > max_distance) {
            other_mallet_current_state.head(3) = current_mallet_state.head(3) + (other_mallet_current_state.head(3) - current_mallet_state.head(3)) * max_distance / distance;
        }

        // Insert the current state of the other mallet
        mallet_base_trajectories[other_mallet_number].insert(mallet_base_trajectories[other_mallet_number].begin() + i, other_mallet_current_state);
    }

    return mallet_base_trajectories;
}

vector<Mallet> assign_mallets(const vector<Vector4f>& notes) {
    vector<Mallet> mallet_assignment;

    for (int i = 0; i < notes.size(); ++i) {
        if (i == 0) {
            if (notes[i](0) < 0) {
                mallet_assignment.push_back(Mallet::LEFT);
            } else {
                mallet_assignment.push_back(Mallet::RIGHT);
            }
            continue;
        }

        if (i < notes.size() - 1 && notes[i + 1](3) == notes[i](3)) {
            if (notes[i](0) < notes[i + 1](0)) {
                mallet_assignment.push_back(Mallet::LEFT);
                mallet_assignment.push_back(Mallet::RIGHT);
            } else {
                mallet_assignment.push_back(Mallet::RIGHT);
                mallet_assignment.push_back(Mallet::LEFT);
            }
            ++i;
            continue;
        }

        if (notes[i](0) < notes[i - 1](0)) {
            mallet_assignment.push_back(Mallet::LEFT);
        } else {
            mallet_assignment.push_back(Mallet::RIGHT);
        }
    }

    return mallet_assignment;
}

map<Mallet, vector<Vector4f>> split_notes_based_on_mallet(const vector<Vector4f>& notes, const vector<Mallet>& mallet_assignment) {
    map<Mallet, vector<Vector4f>> notes_per_mallet = {
        {Mallet::LEFT, {}},
        {Mallet::RIGHT, {}}};

    for (int i = 0; i < notes.rows(); ++i) {
        Mallet mallet = mallet_assignment[i];
        notes_per_mallet[mallet].push_back(notes[i]);
    }

    return notes_per_mallet;
}
} // namespace marimbabot_planning

