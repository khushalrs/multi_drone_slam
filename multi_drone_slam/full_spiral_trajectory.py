import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import - ensures 3D plotting


def circle_x(theta, center, flight_radius):
    """Compute the x coordinate on a circle at angle theta."""
    cx, _ = center
    return cx + flight_radius * np.cos(theta)


def circle_y(theta, center, flight_radius):
    """Compute the y coordinate on a circle at angle theta."""
    _, cy = center
    return cy + flight_radius * np.sin(theta)


def compute_rpy_to_center(position, center):
    """
    Compute roll, pitch, yaw so the drone faces the cylinder center line horizontally.

    Args:
        position (tuple): Current drone position as (x, y, z).
        center (tuple): Horizontal center of the cylinder as (cx, cy).

    Returns:
        (roll, pitch, yaw) in radians.
    """
    x, y, _ = position
    cx, cy = center

    dx = cx - x
    dy = cy - y
    yaw = math.atan2(dy, dx)

    roll = 0.0
    pitch = 0.0
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
        return path_xyzrpy[:1].copy()

    epsilon = 1e-6
    keep_mask = np.ones(len(cumulative_s), dtype=bool)
    keep_mask[1:] = np.diff(cumulative_s) > epsilon
    keep_mask[-1] = True

    unique_s = cumulative_s[keep_mask]
    unique_path = path_xyzrpy[keep_mask]
    unique_xyz = unique_path[:, :3]
    unique_roll = unique_path[:, 3]
    unique_pitch = unique_path[:, 4]
    unique_yaw_unwrapped = np.unwrap(unique_path[:, 5])

    targets = list(np.arange(0.0, total_length, spacing_m))
    if len(targets) == 0 or abs(targets[-1] - total_length) > epsilon:
        targets.append(total_length)

    targets = np.asarray(targets)

    new_xyz = np.column_stack([
        np.interp(targets, unique_s, unique_xyz[:, dim])
        for dim in range(3)
    ])
    new_roll = np.interp(targets, unique_s, unique_roll)
    new_pitch = np.interp(targets, unique_s, unique_pitch)
    new_yaw_unwrapped = np.interp(targets, unique_s, unique_yaw_unwrapped)

    new_yaw = (new_yaw_unwrapped + np.pi) % (2.0 * np.pi) - np.pi
    resampled = np.column_stack((new_xyz, new_roll, new_pitch, new_yaw))
    return resampled


def generate_full_spiral_trajectory(
    center,
    flight_radius,
    total_height,
    num_sweeps,
    points_per_sweep,
    start_angle=0.0,
    min_height=5.0,
    flip_x=False,
    flip_y=False,
):
    """
    Generate a continuous spiral trajectory that wraps fully around the cylinder.

    Args:
        center (tuple): (cx, cy) center of the cylinder.
        flight_radius (float): Distance from the center.
        total_height (float): Target altitude at the end of the spiral.
        num_sweeps (int): Number of full 360° turns in the spiral.
        points_per_sweep (int): Sampling density per turn.
        start_angle (float): Starting angle in radians.
        min_height (float): Height reached before beginning the spiral motion.
        flip_x (bool): Mirror the path about the x-axis through the center.
        flip_y (bool): Mirror the path about the y-axis through the center.

    Returns:
        np.ndarray: (N, 3) array of waypoints (x, y, z).
    """
    cx, cy = center

    if num_sweeps <= 0 or points_per_sweep <= 0:
        return np.zeros((0, 3))

    # Initial vertical ascent to the minimum height.
    initial_points = 20
    z_initial = np.linspace(0.0, min_height, initial_points)
    x_initial = np.full(initial_points, circle_x(start_angle, center, flight_radius))
    y_initial = np.full(initial_points, circle_y(start_angle, center, flight_radius))
    trajectory_parts = [np.column_stack((x_initial, y_initial, z_initial))]

    total_points = num_sweeps * points_per_sweep
    theta_end = start_angle + 2.0 * np.pi * num_sweeps

    theta = np.linspace(start_angle, theta_end, total_points, endpoint=True)
    z_values = np.linspace(min_height, total_height, total_points, endpoint=True)

    x_values = circle_x(theta, center, flight_radius)
    y_values = circle_y(theta, center, flight_radius)

    trajectory_parts.append(np.column_stack((x_values, y_values, z_values)))
    trajectory = np.vstack(trajectory_parts)

    if flip_x:
        trajectory[:, 0] = 2 * cx - trajectory[:, 0]
    if flip_y:
        trajectory[:, 1] = 2 * cy - trajectory[:, 1]

    return trajectory


def plot_trajectory(trajectory):
    """Visualize the 3D trajectory."""
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    x = trajectory[:, 0]
    y = trajectory[:, 1]
    z = trajectory[:, 2]

    ax.plot(x, y, z, label='Full Spiral Trajectory', linewidth=2)
    ax.scatter(x[0], y[0], z[0], c='green', marker='o', s=100, label='Start')
    ax.scatter(x[-1], y[-1], z[-1], c='red', marker='^', s=100, label='End')

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('3D Full Spiral Trajectory')
    ax.legend()
    plt.tight_layout()
    plt.show()


def get_trajectory_with_pose(
    center,
    flight_radius,
    total_height,
    num_sweeps,
    points_per_sweep,
    start_angle=0.0,
    min_height=5.0,
    flip_x=False,
    flip_y=False,
    resample_spacing=0.2,
):
    """
    Generate spiral trajectory samples with associated orientations.

    Returns:
        np.ndarray: (N, 6) array of [x, y, z, roll, pitch, yaw].
    """
    trajectory = generate_full_spiral_trajectory(
        center=center,
        flight_radius=flight_radius,
        total_height=total_height,
        num_sweeps=num_sweeps,
        points_per_sweep=points_per_sweep,
        start_angle=start_angle,
        min_height=min_height,
        flip_x=flip_x,
        flip_y=flip_y,
    )

    num_points = len(trajectory)
    trajectory_with_pose = np.zeros((num_points, 6))
    trajectory_with_pose[:, 0:3] = trajectory

    for idx in range(num_points):
        roll, pitch, yaw = compute_rpy_to_center(trajectory[idx], center)
        trajectory_with_pose[idx, 3:] = [roll, pitch, yaw]

    if resample_spacing is not None:
        trajectory_with_pose = resample_path_equal_arc_length(
            trajectory_with_pose, resample_spacing
        )

    return trajectory_with_pose


if __name__ == "__main__":
    center = (50.0, 20.0)
    flight_radius = 30.0
    total_height = 50.0
    num_sweeps = 6
    points_per_sweep = 100

    traj = get_trajectory_with_pose(
        center=center,
        flight_radius=flight_radius,
        total_height=total_height,
        num_sweeps=num_sweeps,
        points_per_sweep=points_per_sweep,
        min_height=5.0,
        flip_x=False,
        flip_y=False,
        resample_spacing=None,
    )

    plot_trajectory(traj[:, :3])
