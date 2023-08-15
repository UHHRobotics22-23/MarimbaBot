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


def generate_double_trajectory(mallet_1_points, mallet_0_points, mallet_1_timings, mallet_0_timings, down_stroke_speed=2, retreat_speed=2, travel_height=0.2, time_resolution=0.01, max_distance=0.2):
    # Arrange points and timings as 2D arrays for easier manipulation
    mallet_1_points = np.asarray(mallet_1_points)
    mallet_0_points = np.asarray(mallet_0_points)
    mallet_1_timings = np.asarray(mallet_1_timings)
    mallet_0_timings = np.asarray(mallet_0_timings)

    # Get the base trajectories for each mallet
    mallet_0_base_trajectory = generate_base_trajectory(
        mallet_0_points, mallet_0_timings, down_stroke_speed, retreat_speed, travel_height)

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
        if i >= len(mallet_0_base_trajectory) or i >= len(mallet_1_base_trajectory):
            break  # TODO handle this better

        mallet_0_timestamp = mallet_0_base_trajectory[i, 3]
        mallet_1_timestamp = mallet_1_base_trajectory[i, 3]

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
        other_mallet_current_state = other_mallet_previous_state + (other_mallet_next_state - other_mallet_previous_state) * (current_mallet_state[3] - other_mallet_previous_state[3]) / (other_mallet_next_state[3] - other_mallet_previous_state[3])

        # Add another point to the trajectory of the other mallet so that both mallets have the same number of points at the same points in time
        # Check the distance between the two mallets and move the other mallet closer to the current mallet if the distance is exceeded
        distance = np.linalg.norm(current_mallet_state[:3] - other_mallet_current_state[:3])

        if distance > max_distance:
            # Move the other mallet closer to the current mallet so that it is positioned at the maximum distance on the line between the two mallets
            # other_mallet_current_state = current_mallet_state[:3] + (other_mallet_current_state[:3] - current_mallet_state[:3]) * max_distance / distance
            # Keep the z position of the other mallet the same and change the distance by changing the x and y positions
            other_mallet_current_state[:2] =

            other_mallet_current_state = np.hstack((other_mallet_current_state, current_mallet_state[3]))

        # Insert the new state of the other mallet into the trajectory
        mallet_base_trajectories[other_mallet_number] = np.insert(mallet_base_trajectories[other_mallet_number], i, other_mallet_current_state, axis=0)


    # Create the trajectory with linear interpolation
    mallet_trajectories = [np.zeros((0, 4)), np.zeros((0, 4))]
    for mallet, mallet_base_trajectory in enumerate(mallet_base_trajectories):
        for i in range(len(mallet_base_trajectory) - 1):
            # Calculate the number of points to interpolate between the current and next point
            n_points = int((mallet_base_trajectory[i + 1, 3] - mallet_base_trajectory[i, 3]) / time_resolution)

            n_points = 1

            # Create the interpolated points
            interpolated_points = np.linspace(mallet_base_trajectory[i], mallet_base_trajectory[i + 1], n_points)

            # Add the interpolated points to the trajectory
            mallet_trajectories[mallet] = np.vstack((mallet_trajectories[mallet], interpolated_points))

    return mallet_trajectories



if __name__ == "__main__":
    # Define the points and timings in 3D (x, y, z)
    hit_points_mallet_0 = np.array([
        [0.0, 0.0, 0.0],
        [0.5, 0.0, 0.0],
        [1.5, 0.5, 0.0],
        [2.0, 0.0, 0.0],
    ])
    hit_timings_mallet_0 = np.array([0.0, 0.5, 5.0, 5.5])

    hit_points_mallet_1 = np.array([
        [0.0, 0.1, 0.0],
        [0.5, 0.1, 0.0],
    #    [1.5, 0.6, 0.0],
        [2.0, 0.1, 0.0],
    ])
    hit_timings_mallet_1 = np.array([
        0.0,
        0.5,
    #    5.0,
        5.5
    ])

    trajectories = generate_double_trajectory(hit_points_mallet_0, hit_points_mallet_1, hit_timings_mallet_0, hit_timings_mallet_1)

    print(trajectories)

    # Plot the trajectory
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    for trajectory in trajectories:
        # Same but with a color gradient
        ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], marker='o', markersize=2, linestyle='--')

    # Use same scale for all axis
    max_range = np.array([trajectory[:, 0].max()-trajectory[:, 0].min(), trajectory[:, 1].max()-trajectory[:, 1].min(), trajectory[:, 2].max()-trajectory[:, 2].min()]).max() / 2.0
    mid_x = (trajectory[:, 0].max()+trajectory[:, 0].min()) * 0.5
    mid_y = (trajectory[:, 1].max()+trajectory[:, 1].min()) * 0.5
    mid_z = (trajectory[:, 2].max()+trajectory[:, 2].min()) * 0.5
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    plt.show()