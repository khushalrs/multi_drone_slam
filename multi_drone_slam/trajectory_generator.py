import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # Ensures 3D plotting capabilities

# Mathematical functions for the horizontal (circular) coordinates.
def circle_x(theta, center, flight_radius):
    """Compute x coordinate on a circle at angle theta."""
    cx, _ = center
    return cx + flight_radius * np.cos(theta)

def circle_y(theta, center, flight_radius):
    """Compute y coordinate on a circle at angle theta."""
    _, cy = center
    return cy + flight_radius * np.sin(theta)

def vertical_z(theta, base_z, altitude_amplitude, altitude_frequency):
    """Compute z coordinate with altitude variation (sinusoidal) along the trajectory.
    
    For the zigzag, however, we use linear interpolation across each sweep. This function
    is provided for scenarios where you want to use a sinusoidal function along the angle,
    if that was preferred.
    
    Here:
      z = base_z + altitude_amplitude * sin(altitude_frequency * theta)
    """
    return base_z + altitude_amplitude * np.sin(altitude_frequency * theta)

def compute_rpy_to_center(position, center):
    """
    Compute roll, pitch, yaw so that the drone's forward axis points toward a point on the cylinder's center line.

    Args:
        position: Current drone position as (x, y, z).
        center: Horizontal center of the cylinder as (cx, cy).
        target_height: The z-coordinate of the point on the center line to look at.

    Returns:
        (roll, pitch, yaw) in radians.
        - roll is set to zero (assumes no banking).
        - pitch > 0 means nose up; pitch < 0 nose down.
        - yaw is measured from the +X axis toward +Y.
    """
    x, y, z = position
    cx, cy = center

    # Vector from drone to target point
    dx = cx - x
    dy = cy - y

    # Yaw: angle in horizontal plane
    yaw = math.atan2(dy, dx)

    # Roll: assume zero to keep the drone level
    roll, pitch = 0.0, 0.0

    return roll, pitch, yaw


def resample_path_equal_arc_length(path_xyzrpy, spacing_m):
    """
    Resample a pose path to approximately constant 3D spacing.

    Args:
        path_xyzrpy (np.ndarray): Original path as Nx6 array.
        spacing_m (float): Desired spacing in meters between successive samples.

    Returns:
        np.ndarray: Resampled Mx6 pose path.
    """
    if path_xyzrpy is None:
        return None

    if len(path_xyzrpy) == 0:
        return path_xyzrpy

    if spacing_m is None or spacing_m <= 0.0:
        return path_xyzrpy.copy()

    xyz = path_xyzrpy[:, :3]
    diffs = np.diff(xyz, axis=0)
    segment_lengths = np.linalg.norm(diffs, axis=1)
    cumulative_s = np.concatenate(([0.0], np.cumsum(segment_lengths)))
    total_length = cumulative_s[-1]

    if total_length <= 1e-6:
        # Path has no length; collapse to single point copy.
        return path_xyzrpy[:1].copy()

    # Remove duplicated samples that create zero-length segments.
    epsilon = 1e-6
    keep_mask = np.ones(len(cumulative_s), dtype=bool)
    keep_mask[1:] = np.diff(cumulative_s) > epsilon
    keep_mask[-1] = True  # Always keep the final point
    unique_s = cumulative_s[keep_mask]
    unique_path = path_xyzrpy[keep_mask]
    unique_xyz = unique_path[:, :3]
    unique_roll = unique_path[:, 3]
    unique_pitch = unique_path[:, 4]
    unique_yaw_unwrapped = np.unwrap(unique_path[:, 5])

    # Build target arc-length samples.
    targets = list(np.arange(0.0, total_length, spacing_m))
    if len(targets) == 0 or abs(targets[-1] - total_length) > epsilon:
        targets.append(total_length)

    targets = np.asarray(targets)

    # Linear interpolation for position and orientation (yaw unwrapped).
    new_xyz = np.column_stack(
        [
            np.interp(targets, unique_s, unique_xyz[:, dim])
            for dim in range(3)
        ]
    )
    new_roll = np.interp(targets, unique_s, unique_roll)
    new_pitch = np.interp(targets, unique_s, unique_pitch)
    new_yaw_unwrapped = np.interp(targets, unique_s, unique_yaw_unwrapped)

    # Wrap yaw back to [-pi, pi] for output.
    new_yaw = (new_yaw_unwrapped + np.pi) % (2.0 * np.pi) - np.pi

    resampled = np.column_stack((new_xyz, new_roll, new_pitch, new_yaw))
    return resampled

def generate_vertical_zigzag_trajectory(center, 
        flight_radius, total_height, num_sweeps, 
        points_per_sweep, start_angle=0, min_height=5.0, 
        flip_x=False, flip_y=False):
    """
    Generate a trajectory that progresses in a vertical zigzag along a semicircular path.
    
    Args:
        center (tuple): (cx, cy) center of the cylinder.
        flight_radius (float): Distance from the center at which the drone flies.
        total_height (float): Total vertical distance to cover.
        num_sweeps (int): Number of semicircular sweeps (zigzags).
        points_per_sweep (int): Number of waypoints per sweep.
        start_angle (float): Starting angle (in radians) for the first sweep.
        min_height (float): Initial minimum height to reach before starting zigzag.
    
    Returns:
        np.ndarray: An (N, 3) array of waypoints (x, y, z).
    """
    cx, cy = center
    height_per_sweep = (total_height - min_height) / num_sweeps
    trajectory_points = []
    
    # Add initial vertical ascent to min_height
    initial_points = 20  # Number of points for initial ascent
    z_initial = np.linspace(0, min_height, initial_points)
    x_initial = np.full(initial_points, circle_x(start_angle, center, flight_radius))
    y_initial = np.full(initial_points, circle_y(start_angle, center, flight_radius))
    initial_ascent = np.column_stack((x_initial, y_initial, z_initial))
    trajectory_points.append(initial_ascent)
    
    for sweep in range(num_sweeps):
        # Determine start and end altitude for this sweep
        z_start = min_height + sweep * height_per_sweep
        z_end = min_height + (sweep + 1) * height_per_sweep
        # Create linearly spaced z values for this sweep
        z_values = np.linspace(z_start, z_end, points_per_sweep)
        
        # Compute theta for a semicircular arc (pi radians) and reverse on every other sweep
        if sweep % 2 == 0:
            theta = np.linspace(start_angle, start_angle + np.pi, points_per_sweep)
        else:
            theta = np.linspace(start_angle + np.pi, start_angle, points_per_sweep)
        
        # Compute x and y coordinates using vectorized calls
        x_values = circle_x(theta, center, flight_radius)
        y_values = circle_y(theta, center, flight_radius)
        
        # Stack into a (points_per_sweep, 3) array
        sweep_points = np.column_stack((x_values, y_values, z_values))
        trajectory_points.append(sweep_points)
    
    # 3) Concatenate
    trajectory = np.vstack(trajectory_points)
    
    # 4) Optional mirror
    if flip_x:
        trajectory[:,0] = 2*cx - trajectory[:,0]
    if flip_y:
        trajectory[:,1] = 2*cy - trajectory[:,1]
    return trajectory

# Visualization: Plot the 3D trajectory using Matplotlib.
def plot_trajectory(trajectory):
    """
    Visualize the trajectory in 3D.
    
    Args:
        trajectory (np.ndarray): An (N, 3) array of waypoints.
    """
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    x = trajectory[:, 0]
    y = trajectory[:, 1]
    z = trajectory[:, 2]
    
    # Plot the trajectory line.
    ax.plot(x, y, z, label='Zigzag Trajectory', linewidth=2)
    
    # Optionally, mark the start and end points.
    ax.scatter(x[0], y[0], z[0], c='green', marker='o', s=100, label='Start')
    ax.scatter(x[-1], y[-1], z[-1], c='red', marker='^', s=100, label='End')
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('3D Zigzag Trajectory along a Semicircular Path')
    ax.legend()
    plt.tight_layout()
    plt.show()

def get_trajectory_with_pose(center, flight_radius, 
        total_height, num_sweeps, points_per_sweep, 
        start_angle=0, min_height=5.0, flip_x=False, 
        flip_y=False, resample_spacing=0.2):
    """
    Generate a trajectory with position and orientation information.

    Args:
        center (tuple): (cx, cy) center of the cylinder.
        flight_radius (float): Distance from the center at which the drone flies.
        total_height (float): Total vertical distance to cover.
        num_sweeps (int): Number of semicircular sweeps (zigzags).
        points_per_sweep (int): Number of waypoints per sweep.
        start_angle (float): Starting angle (in radians) for the first sweep.
        min_height (float): Initial minimum height to reach before starting zigzag.

    Returns:
        np.ndarray: An (N, 6) array containing [x, y, z, roll, pitch, yaw] for each point.
    """
    # Generate the base trajectory
    trajectory = generate_vertical_zigzag_trajectory(
        center, 
        flight_radius, 
        total_height, 
        num_sweeps, 
        points_per_sweep,
        start_angle,
        flip_x=flip_x,
        flip_y=flip_y
    )
    # print("Trajectory shape:", trajectory)
    
    num_points = len(trajectory)
    trajectory_with_pose = np.zeros((num_points, 6))
    
    # Copy position coordinates
    trajectory_with_pose[:, 0:3] = trajectory
    
    # Compute orientation for each point
    for i in range(num_points):
        roll, pitch, yaw = compute_rpy_to_center(trajectory[i], center)
        trajectory_with_pose[i, 3:] = [roll, pitch, yaw]

    if resample_spacing is not None:
        trajectory_with_pose = resample_path_equal_arc_length(
            trajectory_with_pose, resample_spacing
        )

    return trajectory_with_pose

# Example usage:
if __name__ == '__main__':
    # Define trajectory parameters:
    center = (50.0, 0.0)          # Center of the semicircular path (cylinder center)
    flight_radius = 23.0          # Desired distance from the center
    total_height = 50.0          # Total vertical distance to cover
    num_sweeps = 4               # Number of zigzag passes (semicircular sweeps)
    points_per_sweep = 20        # Resolution of each sweep
    
    # Generate the zigzag trajectory.
    trajectory = get_trajectory_with_pose(
        center, 
        flight_radius, 
        total_height, 
        num_sweeps, 
        points_per_sweep,
        min_height=5.0,
        flip_x=True,
        flip_y=False
    )
    # print("Trajectory shape:", trajectory[15:25])
    
    # # Print trajectory points and corresponding poses
    # print("\nTrajectory Points and Poses:")
    # print("Point\t\t\t\tRoll\tPitch\tYaw")
    # print("-" * 50)
    
    # for point in trajectory:
    #     x, y, z = point
    #     roll, pitch, yaw = compute_rpy_to_center((x, y, z), center)
    #     print(f"({x:.2f}, {y:.2f}, {z:.2f})\t{roll:.2f}\t{pitch:.2f}\t{yaw:.2f}")
    
    # # Visualize the resulting trajectory in 3D.
    plot_trajectory(trajectory)
