from enum import Enum

import numpy as np
import matplotlib.pyplot as plt


def generate_base_trajectory(points, timings, down_stroke_speed, retreat_speed, travel_height):
    """
    Generates the base trajectory for a single mallet.
    The base trajectory consists only of the key points and their respective timings.
    No interpolation is performed.
    Primarlily the down strokes are are constructed from the hit points and timings.
    """
    base_trajectory_points = []
    base_trajectory_timings = []

    for i in range(len(points)):
        approach_point = points[i] + np.array([0.0, 0.0, travel_height])
        retreat_point = points[i] + np.array([0.0, 0.0, travel_height])
        base_trajectory_points.extend([approach_point, points[i], retreat_point])
        base_trajectory_timings.extend([
            timings[i] - travel_height / down_stroke_speed,
            timings[i],
            timings[i] + travel_height / retreat_speed])  # TODO handle possible negative times


    # Add the timings so that each point has a time associated with it using numpy
    return np.hstack((
        np.array(base_trajectory_points), np.array(base_trajectory_timings)[:, np.newaxis]))


def generate_double_trajectory(mallet_0_notes, mallet_1_notes, down_stroke_speed=2, retreat_speed=2, travel_height=0.2, time_resolution=0.01, max_distance=0.2):
    # Arrange points and timings as 2D arrays for easier manipulation
    mallet_0_points = np.asarray(mallet_0_notes)[..., :3]
    mallet_1_points = np.asarray(mallet_1_notes)[..., :3]
    mallet_0_timings = np.asarray(mallet_0_notes)[..., 3]
    mallet_1_timings = np.asarray(mallet_1_notes)[..., 3]

    # Get the base trajectories for each mallet
    mallet_0_base_trajectory = generate_base_trajectory(
        mallet_0_points, mallet_0_timings, down_stroke_speed, retreat_speed, travel_height)

    print(mallet_0_base_trajectory)

    mallet_1_base_trajectory = generate_base_trajectory(
        mallet_1_points, mallet_1_timings, down_stroke_speed, retreat_speed, travel_height)

    mallet_base_trajectories = [mallet_0_base_trajectory, mallet_1_base_trajectory]


    # Find the number of unique timestamps in the combined trajectory
    num_unique_timestamps = len(np.unique(np.hstack((mallet_0_base_trajectory[..., 3], mallet_1_base_trajectory[..., 3]))))

    # Iterate over all points in the combined trajectory and check if the distance between the two mallets is exceeded
    # To do this, create a new point for the mallet that is not assosiated with the current point.
    # If the distance between the two mallets is exceeded, move the mallet that is not assosiated with the current point closer to the other mallet.
    # Get the state of the mallet not assosiated with the current point by interpolating between the previous and next point linearly.
    for i in range(num_unique_timestamps):

        # Check if i is out of bounds for one of the mallets and if so break
        if i >= len(mallet_base_trajectories[0]) or i >= len(mallet_base_trajectories[1]):
            break  # TODO handle this better

        mallet_0_timestamp = mallet_base_trajectories[0][i, 3]
        mallet_1_timestamp = mallet_base_trajectories[1][i, 3]

        if mallet_0_timestamp < mallet_1_timestamp:
            current_mallet = 0
            other_mallet_number = 1
        elif mallet_0_timestamp > mallet_1_timestamp:
            current_mallet = 1
            other_mallet_number = 0
        else:
            # If the timestamps are equal, both mallets are allready fully defined for this timestamp and we can skip to the next timestamp
            continue

        current_mallet_state = mallet_base_trajectories[current_mallet][i]
        other_mallet_previous_state = mallet_base_trajectories[other_mallet_number][max(i - 1, 0)]
        other_mallet_next_state = mallet_base_trajectories[other_mallet_number][i]

        # Calculate the current state of the other mallet by interpolating between the previous and next state linearly
        if np.all(other_mallet_previous_state == other_mallet_next_state):
            other_mallet_current_state = other_mallet_next_state.copy()
            other_mallet_current_state[3] = current_mallet_state[3]
        else:
            other_mallet_current_state = other_mallet_previous_state + (other_mallet_next_state - other_mallet_previous_state) * (current_mallet_state[3] - other_mallet_previous_state[3]) / (other_mallet_next_state[3] - other_mallet_previous_state[3])

        # Add another point to the trajectory of the other mallet so that both mallets have the same number of points at the same points in time
        # Check the distance between the two mallets and move the other mallet closer to the current mallet if the distance is exceeded
        distance = np.linalg.norm(current_mallet_state[:3] - other_mallet_current_state[:3])

        if distance > max_distance:
            # Move the other mallet closer to the current mallet so that it is positioned at the maximum distance on the line between the two mallets
            other_mallet_current_state[:3] = current_mallet_state[:3] + (other_mallet_current_state[:3] - current_mallet_state[:3]) * max_distance / distance

        print(mallet_base_trajectories[0] ) if other_mallet_number == 0 else None

        # Insert the new state of the other mallet into the trajectory
        mallet_base_trajectories[other_mallet_number] = np.insert(mallet_base_trajectories[other_mallet_number], i, other_mallet_current_state, axis=0)

    return mallet_base_trajectories


class Mallet(Enum):
    LEFT = 0
    RIGHT = 1


def assign_mallets(notes):
    """
    Assigns a mallet to each note. We need to take care that the mallets dont cross each other.
    The assignment is done greedily
    """

    # A list that maps each note to a mallet
    mallet_assignment: list(Mallet) = []


    # Iterate over the rest of the notes and assign a mallet to each note
    # We explicitly handle the indexing as we might increment the index by more than one
    i = 0
    while i < len(notes):
        note = notes[i]
        # Explicit handling of the first note
        if i == 0:
            # Start with the left mallet if the first note is on the left side of the marimba
            if note[0] < 0:
                mallet_assignment.append(Mallet.LEFT)
            else:
                mallet_assignment.append(Mallet.RIGHT)
            i += 1
            continue

        # Check if the next two notes are a chord
        if i < len(notes) - 1 and notes[i + 1][3] == note[3]:
            # The next two notes are a chord
            # Assign the left mallet to the left note and the right mallet to the right note
            # Check if the current note is on the left or right side of the previous note
            if note[0] < notes[i + 1][0]:
                # The current note is on the left side of the next note
                # Use the left mallet for the current note
                mallet_assignment.append(Mallet.LEFT)
                # Use the right mallet for the next note
                mallet_assignment.append(Mallet.RIGHT)
            else:
                # The current note is on the right side of the next note
                # Use the right mallet for the current note
                mallet_assignment.append(Mallet.RIGHT)
                # Use the left mallet for the next note
                mallet_assignment.append(Mallet.LEFT)
            i += 2
            continue

        # Check if the next note if on the left or right side of the previous note
        if note[0] < notes[i - 1][0]:
            print(note, notes[i - 1])
            # The next note is on the left side of the previous note
            # Use the left mallet
            mallet_assignment.append(Mallet.LEFT)
        else:
            # The next note is on the right side of the previous note
            # Use the right mallet
            mallet_assignment.append(Mallet.RIGHT)

        # increment the index
        i += 1

    return mallet_assignment


def split_notes_based_on_mallet(notes, mallet_asignment: list(Mallet)):
    """
    Splits the notes into two lists based on the mallet assignment
    """

    # A list that contains the notes for each mallet
    notes_per_mallet = [[], []]

    # Iterate over the notes and assign them to the correct list
    for i, note in enumerate(notes):
        # Get the mallet for the current note
        mallet = mallet_asignment[i]

        # Add the note to the list of the correct mallet
        notes_per_mallet[mallet.value].append(note)

    return notes_per_mallet


if __name__ == "__main__":
    # Define the points and timings (x, y, z, time)
    notes = np.array([
        [0.0, 0.3, 0.0, 0.0],
        [0.5, 0.0, 0.0, 1.0],
        [0.0, 0.0, 0.0, 2.0],
        [2.0, 0.0, 0.0, 3.0],
        [0.0, 0.3, 0.0, 4.0],
        [0.1, 0.3, 0.0, 4.0],
    ])

    # Assign the mallets to the notes
    mallet_assignment = assign_mallets(notes)

    print(mallet_assignment)

    # Split the notes based on the mallet assignment
    mallet_0_notes, mallet_1_notes = split_notes_based_on_mallet(notes, mallet_assignment)

    print(mallet_0_notes, mallet_1_notes)

    trajectories = generate_double_trajectory(mallet_0_notes, mallet_1_notes)

    print(trajectories)

    # Plot the trajectory
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    color_mapping = ['red', 'blue']

    for i, trajectory in enumerate(trajectories):
        # Same but with a color gradient
        ax.plot(
            trajectory[:, 0],
            trajectory[:, 1],
            trajectory[:, 2],
            marker='o',
            markersize=2,
            linestyle='--',
            color=color_mapping[i],
            label=f"{Mallet(i).name} Mallet")

    # Show legend
    ax.legend()

    # Use same scale for all axis note that there are multiple trajectories
    max_range = np.array([trajectory[:, :3].max(axis=0) - trajectory[:, :3].min(axis=0) for trajectory in trajectories]).max()
    mean_x = np.array([trajectory[:, 0].mean() for trajectory in trajectories]).mean()
    mean_y = np.array([trajectory[:, 1].mean() for trajectory in trajectories]).mean()
    mean_z = np.array([trajectory[:, 2].mean() for trajectory in trajectories]).mean()
    ax.set_xlim(mean_x - max_range, mean_x + max_range)
    ax.set_ylim(mean_y - max_range, mean_y + max_range)
    ax.set_zlim(mean_z - max_range, mean_z + max_range)


    plt.show()
