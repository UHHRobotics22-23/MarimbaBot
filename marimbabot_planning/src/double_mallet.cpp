#include "marimbabot_planning/double_mallet.h"

namespace marimbabot_planning
{

std::vector<Eigen::Vector4f> generate_base_trajectory(const std::vector<CartesianHitSequenceElement>& hit_points, float travel_height) {
    std::vector<Eigen::Vector4f> result;

    double fastest_down_stroke_duration = 0.5;
    double most_silent_hit_duration = 1.0;
    double retreat_speed = 0.1;

    double last_note_end = -1000;

    for (const auto& hit_point : hit_points) {
        double loudness = hit_point.msg.loudness;

        // Approach point time
        double approach_point_time = hit_point.msg.start_time.toSec() - fastest_down_stroke_duration - (1 - loudness) * most_silent_hit_duration;

        // Check if the approach time is before the current time
        if (approach_point_time < last_note_end) {
            // Throw an error
            throw std::runtime_error("Notes are overlapping and cannot be played at the same time, please play slower");
            // TODO do an exception that is handled by the action server response
        }

        // Create the approach point
        result.push_back(Eigen::Vector4f(
            hit_point.point.point.x,
            hit_point.point.point.y,
            hit_point.point.point.z + travel_height,
            approach_point_time));

        // Copy over the hit point
        result.push_back(Eigen::Vector4f(
            hit_point.point.point.x,
            hit_point.point.point.y,
            hit_point.point.point.z,
            hit_point.msg.start_time.toSec()));

        // Create the retreat point
        result.push_back(Eigen::Vector4f(
            hit_point.point.point.x,
            hit_point.point.point.y,
            hit_point.point.point.z + travel_height,
            hit_point.msg.start_time.toSec() + retreat_speed));

        // Update the current time
        last_note_end = hit_point.msg.start_time.toSec() + retreat_speed;
    }

    return result;
}

std::vector<DoubleMalletKeyframe> generate_double_trajectory(const std::vector<CartesianHitSequenceElement>& hit_points, float travel_height, float time_resolution, float max_distance) {

    // Get the planning frame_id from the first hit point
    std::string planning_frame_id = hit_points[0].point.header.frame_id;

    // Assign each hit point a mallet
    std::vector<Mallet> mallet_assignment = assign_mallets(hit_points);

    // Split the hit points into two vectors based on the mallet assignment
    auto split_hit_points = split_notes_based_on_mallet(hit_points, mallet_assignment);

    // Create a mapping from mallet (enum) to base trajectory
    std::map<Mallet, std::vector<Eigen::Vector4f>> mallet_base_trajectories = {
        {Mallet::LEFT, generate_base_trajectory(split_hit_points[Mallet::LEFT], travel_height)},
        {Mallet::RIGHT, generate_base_trajectory(split_hit_points[Mallet::RIGHT], travel_height)}};

    // Find the maximum possible (but improbable) number of timestamps (the sum of the two trajectories)
    int maximum_number_of_timestamps = mallet_base_trajectories[Mallet::LEFT].size() + mallet_base_trajectories[Mallet::RIGHT].size();

    for (int i = 0; i < maximum_number_of_timestamps; ++i) {
        // Check if we've reached the end of one of the trajectories (no more double planning is needed for the rest)
        if (i >= mallet_base_trajectories[Mallet::LEFT].size() || i >= mallet_base_trajectories[Mallet::RIGHT].size()) {
            break;
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

    // Convert to keyframes
    std::vector<DoubleMalletKeyframe> result;

    int longest_trajectory = std::max(
        mallet_base_trajectories[Mallet::LEFT].size(),
        mallet_base_trajectories[Mallet::RIGHT].size());

    // Determine the mallet with the longest trajectory
    Mallet longest_trajectory_mallet = mallet_base_trajectories[Mallet::LEFT].size() > mallet_base_trajectories[Mallet::RIGHT].size() ? Mallet::LEFT : Mallet::RIGHT;

    // Store the the of the last keyframe while iterating
    double current_time = mallet_base_trajectories[longest_trajectory_mallet][0](3);

    for (int i = 0; i < longest_trajectory; i++) {
        DoubleMalletKeyframe keyframe;

        if (i < mallet_base_trajectories[Mallet::LEFT].size()) {
            keyframe.left_mallet_active = true;
            keyframe.left_mallet_position.point.x = mallet_base_trajectories[Mallet::LEFT][i](0);
            keyframe.left_mallet_position.point.y = mallet_base_trajectories[Mallet::LEFT][i](1);
            keyframe.left_mallet_position.point.z = mallet_base_trajectories[Mallet::LEFT][i](2);
            keyframe.left_mallet_position.header.frame_id = planning_frame_id;
            keyframe.duration = mallet_base_trajectories[Mallet::LEFT][i](3) - current_time;
        }

        if (i < mallet_base_trajectories[Mallet::RIGHT].size()) {
            keyframe.right_mallet_active = true;
            keyframe.right_mallet_position.point.x = mallet_base_trajectories[Mallet::RIGHT][i](0);
            keyframe.right_mallet_position.point.y = mallet_base_trajectories[Mallet::RIGHT][i](1);
            keyframe.right_mallet_position.point.z = mallet_base_trajectories[Mallet::RIGHT][i](2);
            keyframe.right_mallet_position.header.frame_id = planning_frame_id;
            keyframe.duration = mallet_base_trajectories[Mallet::RIGHT][i](3) - current_time;
        }

        current_time = keyframe.duration + current_time;

        result.push_back(keyframe);
    }
    
    return result;
}

std::vector<Mallet> assign_mallets(const std::vector<CartesianHitSequenceElement>& notes) {
    std::vector<Mallet> mallet_assignment;

    for (int i = 0; i < notes.size(); ++i) {
        if (i == 0) {
            if (notes[i].point.point.x < 0) {
                mallet_assignment.push_back(Mallet::LEFT);
            } else {
                mallet_assignment.push_back(Mallet::RIGHT);
            }
            continue;
        }

        if (i < notes.size() - 1 && notes[i + 1].msg.start_time == notes[i].msg.start_time) {
            if (notes[i].point.point.x < notes[i + 1].point.point.x) {
                mallet_assignment.push_back(Mallet::LEFT);
                mallet_assignment.push_back(Mallet::RIGHT);
            } else {
                mallet_assignment.push_back(Mallet::RIGHT);
                mallet_assignment.push_back(Mallet::LEFT);
            }
            ++i;
            continue;
        }

        if (notes[i].point.point.x < notes[i - 1].point.point.x) {
            mallet_assignment.push_back(Mallet::LEFT);
        } else {
            mallet_assignment.push_back(Mallet::RIGHT);
        }
    }

    return mallet_assignment;
}

std::map<Mallet, std::vector<CartesianHitSequenceElement>> split_notes_based_on_mallet(const std::vector<CartesianHitSequenceElement>& notes, const std::vector<Mallet>& mallet_assignment) {
    std::map<Mallet, std::vector<CartesianHitSequenceElement>> notes_per_mallet = {
        {Mallet::LEFT, {}},
        {Mallet::RIGHT, {}}};

    for (int i = 0; i < notes.size(); ++i) {
        Mallet mallet = mallet_assignment[i];
        notes_per_mallet[mallet].push_back(notes[i]);
    }

    return notes_per_mallet;
}
} // namespace marimbabot_planning
