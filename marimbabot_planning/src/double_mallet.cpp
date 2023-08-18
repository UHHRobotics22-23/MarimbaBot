#include "marimbabot_planning/double_mallet.h"

namespace marimbabot_planning
{

std::vector<Eigen::Vector4f> generate_base_trajectory(const std::vector<Eigen::Vector4f>& hit_points, float down_stroke_speed, float retreat_speed, float travel_height) {
    std::vector<Eigen::Vector4f> result;

    for (int i = 0; i < hit_points.size(); ++i) {
        // Create the approach point
        result.push_back(hit_points[i] + Eigen::Vector4f(0.0, 0.0, travel_height, -travel_height / down_stroke_speed));
        // Copy over the hit point
        result.push_back(hit_points[i]);
        // Create the retreat point
        result.push_back(hit_points[i] + Eigen::Vector4f(0.0, 0.0, travel_height, travel_height / retreat_speed));
    }

    return result;
}

std::map<Mallet, std::vector<Eigen::Vector4f>> generate_double_trajectory(const std::vector<Eigen::Vector4f>& hit_points, float down_stroke_speed, float retreat_speed, float travel_height, float time_resolution, float max_distance) {

    // Assign each hit point a mallet
    std::vector<Mallet> mallet_assignment = assign_mallets(hit_points);

    // Split the hit points into two vectors based on the mallet assignment
    auto split_hit_points = split_notes_based_on_mallet(hit_points, mallet_assignment);

    // Create a mapping from mallet (enum) to base trajectory
    std::map<Mallet, std::vector<Eigen::Vector4f>> mallet_base_trajectories = {
        {Mallet::LEFT, generate_base_trajectory(split_hit_points[Mallet::LEFT], down_stroke_speed, retreat_speed, travel_height)},
        {Mallet::RIGHT, generate_base_trajectory(split_hit_points[Mallet::RIGHT], down_stroke_speed, retreat_speed, travel_height)}};


    int maximum_number_of_timestamps = std::max(
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

        Eigen::Vector4f current_mallet_state = mallet_base_trajectories[current_mallet][i];
        Eigen::Vector4f other_mallet_previous_state = mallet_base_trajectories[other_mallet_number][std::max(i - 1, 0)];
        Eigen::Vector4f other_mallet_next_state = mallet_base_trajectories[other_mallet_number][i];

        Eigen::Vector4f other_mallet_current_state;

        // Check if the other mallet previous state and future state are the same
        if (other_mallet_previous_state(3) == other_mallet_next_state(3)) {
            // Copy over the previous state. Do not pass the reference, as we will modify it!
            other_mallet_current_state = other_mallet_previous_state;
            // Update the timestamp
            other_mallet_current_state(3) = current_mallet_state(3);

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

std::vector<Mallet> assign_mallets(const std::vector<Eigen::Vector4f>& notes) {
    std::vector<Mallet> mallet_assignment;

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

std::map<Mallet, std::vector<Eigen::Vector4f>> split_notes_based_on_mallet(const std::vector<Eigen::Vector4f>& notes, const std::vector<Mallet>& mallet_assignment) {
    std::map<Mallet, std::vector<Eigen::Vector4f>> notes_per_mallet = {
        {Mallet::LEFT, {}},
        {Mallet::RIGHT, {}}};

    for (int i = 0; i < notes.size(); ++i) {
        Mallet mallet = mallet_assignment[i];
        notes_per_mallet[mallet].push_back(notes[i]);
    }

    return notes_per_mallet;
}
} // namespace marimbabot_planning
