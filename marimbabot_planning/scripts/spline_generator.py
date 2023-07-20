import numpy as np
from scipy.interpolate import CubicHermiteSpline, CubicSpline
from scipy.optimize import minimize
import matplotlib.pyplot as plt

def generate_trajectory(points, timings, down_stroke_speed=3, retreat_speed=3, approach_time_offset=0.1, retreat_time_offset=0.1, time_resolution=0.01):
    n_points = len(points)
    
    # Arrange points and timings as 2D arrays for easier manipulation
    points = np.array(points)
    timings = np.array(timings)
    
    # Sort points and timings based on timings
    sorted_indices = np.argsort(timings)
    points = points[sorted_indices]
    timings = timings[sorted_indices]
    
    # Create the trajectory points, including the down stroke and travel points
    trajectory_points = []
    trajectory_timings = []
    trajectory_velocities = []    
    
    for i in range(n_points):
        retreat_point = points[i] + np.array([0.0, 0.0, retreat_speed * retreat_time_offset])
        trajectory_points.extend([points[i], retreat_point])
        trajectory_timings.extend([timings[i], timings[i] + retreat_time_offset])
        trajectory_velocities.extend([[0.0, 0.0, -down_stroke_speed], [0.0, 0.0, retreat_speed]])
    
    trajectory_points = np.array(trajectory_points)
    trajectory_timings = np.array(trajectory_timings)
    trajectory_velocities = np.array(trajectory_velocities)

    print(trajectory_timings)
    
    # Generate spline for each dimension (x, y, z)
    t = np.arange(0, trajectory_timings[-1], time_resolution)
    splines = [CubicHermiteSpline(trajectory_timings, trajectory_points[:, i], trajectory_velocities[:, i]) for i in range(3)]

    # Evaluate the splines to get the final trajectory points
    trajectory = np.array([s(t) for s in splines]).T
    
    return trajectory



def optimize_hit_trajectory(start_point, start_velocity, hit_point, hit_speed, hit_time, retreat_speed, board_clearence=0.2, max_height=0.5, time_resolution=100, num_control_points=2):
    """
    Optimize the trajectory of the mallet to hit a point (note) on the marimba. It considers different objectives during the optimization process. 
    We optimize the control points of the spline, which are the points that define the trajectory of the mallet. 
    """

    def generate_trajectory(control_points):
        # Generate the trajectory using the control points (x, y, z, t)
        control_points = np.array(control_points).reshape(-1, 2, 3)
        control_point_times = np.linspace(0, hit_time + 0.2, num_control_points)
        control_point_positions = control_points[:, 0, :]
        control_point_velocities = control_points[:, 1, :]
        control_point_velocities[-1, :] = np.array([0.0, 0.0, -retreat_speed])

        splines = [CubicHermiteSpline(control_point_times, control_point_positions[:, i], control_point_velocities[:, i]) for i in range(3)]
        
        t = np.arange(0.0, control_point_times[-1], 1/time_resolution)
        trajectory = np.array([s(t) for s in splines]).T

        return trajectory

    # Define the objective function
    def objective_function(control_points):

        # Generate the trajectory using the control points (x, y, z)
        trajectory = generate_trajectory(control_points)

        # Objective 1: Minimize the error in the retreat speed
        retreat_speed_error = np.linalg.norm((trajectory[-1] - trajectory[-2]) - np.array([0.0, 0.0, retreat_speed]))

        print("Retreat speed error: ", np.round((trajectory[-1] - trajectory[-2]), 3))

        # Objective 2: Minimize the error in hit position at the hit time
        hit_position_error = np.linalg.norm(trajectory[min(int(hit_time * time_resolution), len(trajectory) -1)] - hit_point)

        # Objective 3: Minimize the error in hit speed at the hit time
        hit_speed_error = np.linalg.norm(
            trajectory[min(int(hit_time * time_resolution), len(trajectory) - 1)] \
                - trajectory[min(int(hit_time * time_resolution), len(trajectory) - 1) - 1] + np.array([0.0, 0.0, hit_speed]))
           
        # Print the above vector as decimals for better readability
        print("Velocity error: ", np.round(trajectory[min(int(hit_time * time_resolution), len(trajectory) - 1)] \
                - trajectory[min(int(hit_time * time_resolution), len(trajectory) - 1) - 1], 3))
        
        # Objective 4: Minimize the error in the max height of the trajectory
        max_height_error = np.linalg.norm(np.max(trajectory[:, 2]) - max_height)

        # Objective 5: Minimize the error in the start position
        start_position_error = np.linalg.norm(trajectory[0] - start_point)

        # Objective 6: Minimize the error in the start velocity
        start_velocity_error = np.linalg.norm(
            (trajectory[1] - trajectory[0]) * time_resolution - start_velocity)
        
        # Objective 7: Minimize the path length (calculate as the sum of the distance between each pair of points with numpy)
        lengths = np.sqrt(np.sum(np.diff(trajectory, axis=0)**2, axis=1)) # Length between corners
        total_length = np.sum(lengths)

        # Objective 8: Maximize the mean distance between the trajectory and the board (calculate as the mean of the distance between each point and the board)
        board_distance = np.mean((trajectory[:, 2] - board_clearence)**2)
        
        # Calculate the objective function
        objective = 0.0
        objective += 500.0 * retreat_speed_error ** 2
        objective += 2.0 * hit_position_error ** 2
        objective += 500.0 * hit_speed_error ** 2
        objective += 1.0 * max_height_error ** 2
        objective += 1.0 * start_position_error ** 2
        #objective += 1.0 * start_velocity_error ** 2
        objective += 0.01 * total_length ** 2
        objective += 10.0 * board_distance ** 2

        return objective / 10
    
    # Create the initial control points evenly spaced between the start and hit point 
    control_points = np.zeros((num_control_points, 2, 3))
    control_points[:, 0, :] = np.linspace(start_point, hit_point, num_control_points)

    # Optimize the control points
    result = minimize(objective_function, control_points.reshape(-1), tol=1e-5, method='CG', options={'maxiter': 200, 'disp': True})

    # Generate the trajectory using the optimized control points
    trajectory = generate_trajectory(result.x)

    return trajectory, result.x.reshape(-1, 2, 3)


# Example usage:
#if __name__ == "__main__":
#    # Define the points and timings in 3D (x, y, z)
#    points = [
#        [0.0, 0.5, 0.0],
#        [1.0, 0.0, 0.0],
#        [1.5, 0.0, 0.0],
#        [1.5, 0.0, 0.0],
#        [2.0, 0.5, 0.0],
#        [8.0, 0.0, 0.0]
#    ]
#    timings = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]
#    
#    trajectory = generate_trajectory(points, timings)
#    print(trajectory)
#    
#    # Plot the trajectory
#    fig = plt.figure()
#    ax = fig.add_subplot(111, projection='3d')
#    ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], marker='o', markersize=2)
#
#    # Plot the points
#    points = np.array(points)
#    ax.plot(points[:, 0], points[:, 1], points[:, 2], marker='o', markersize=5, color='red')
#
#    plt.show()

if __name__ == "__main__":
    # Define the points and timings in 3D (x, y, z)
    start_point = [0.0, 0.0, 0.0]
    start_velocity = [0.0, 0.0, 0.0]
    hit_point = [1.0, 0.0, 0.0]
    hit_speed = 0.002
    hit_time = 1.0
    retreat_speed = 0.002
    board_clearence = 0.2
    max_height = 0.3
    time_resolution = 100
    num_control_points = 5

    trajectory, control_points = optimize_hit_trajectory(start_point, start_velocity, hit_point, hit_speed, hit_time, retreat_speed, board_clearence, max_height, time_resolution, num_control_points)
    #print(trajectory)

    # Plot the trajectory
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], marker='o', markersize=2)

    # Plot the points
    points = np.array([start_point, hit_point])
    ax.plot(points[:, 0], points[:, 1], points[:, 2], marker='o', markersize=5, color='red')

    # Plot the control points
    ax.plot(control_points[:, 0, 0], control_points[:, 0, 1], control_points[:, 0, 2], marker='o', markersize=5, color='green')

    # Plot velocity arrows
    for i in range(len(control_points)):
        ax.quiver(control_points[i, 0, 0], control_points[i, 0, 1], control_points[i, 0, 2], control_points[i, 1, 0] - control_points[i, 0, 0], control_points[i, 1, 1] - control_points[i, 0, 1], control_points[i, 1, 2] - control_points[i, 0, 2], length=0.1, normalize=False, color='green')

    # Use same scale for all axis
    max_range = np.array([trajectory[:, 0].max()-trajectory[:, 0].min(), trajectory[:, 1].max()-trajectory[:, 1].min(), trajectory[:, 2].max()-trajectory[:, 2].min()]).max() / 2.0
    mid_x = (trajectory[:, 0].max()+trajectory[:, 0].min()) * 0.5
    mid_y = (trajectory[:, 1].max()+trajectory[:, 1].min()) * 0.5
    mid_z = (trajectory[:, 2].max()+trajectory[:, 2].min()) * 0.5
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    plt.show()