import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generates the launch description for a multi-drone SLAM setup.

    This launch file starts the necessary components for two drones, including:
    - A Gazebo to ROS bridge for sensor data.
    - MAVROS for communication with the PX4 autopilot.
    - TF publishers for odom and static transforms.
    """
    pkg_share = get_package_share_directory('multi_drone_slam')
    bridge_config_path = os.path.join(pkg_share, 'config', 'single_lidar_bridge_config.yaml')
    
    # --- SHARED NODES ACROSS ALL DRONES ---
    gz_ros_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_ros_bridge',
        parameters=[{'config_file': bridge_config_path, 'use_sim_time': True}],
        output='screen'
    )

    mavros = Node(
        package='mavros',
        executable='mavros_node',
        output='screen',
        arguments=[
            '--ros-args',
            '-p', 'fcu_url:=udp://:14540@127.0.0.1:14580',
            '-p', 'tgt_system:=1',
            '-p', 'fcu_protocol:=v2.0',
            '-r', '__ns:=/drone1'
        ],
        parameters=[{'use_sim_time': True}]
    )

    # --- Static transforms to connect each drone's map to a global 'map' frame ---
    # This allows visualizing both drones in RViz under a common frame.
    # RTAB-Map will publish the transform from its map_frame (e.g., 'drone1/map')
    # to its odom_frame (e.g., 'drone1/odom').

    # Drone spawns at (0, 0, ...), so its map frame is aligned with the global map frame.
    static_map_to_drone_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_drone_map_publisher',
        parameters= [{'use_sim_time': True}],
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'drone1/map']
    )

    # --- DRONE  SETUP ---
    drone_ns = 'drone1'

    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='drone_base_to_camera_broadcaster',
        namespace=drone_ns,
        parameters=[{'use_sim_time': True}],
        arguments=[
            '0.12', '0.03', '0.02', '0', '0', '0',
            f'{drone_ns}/base_link',
            f'{drone_ns}/camera_link'
        ]
    )

    imu_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='drone_base_to_lidar_broadcaster',
        namespace=drone_ns,
        parameters=[{'use_sim_time': True}],
        arguments=[
            '0.3', '0', '0.01', '0', '0', '0',
            f'{drone_ns}/base_link',
            'my_lidar_camera_drone_0/base_link/imu_sensor'
        ]
    )

    lidar_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='drone_base_to_lidar_broadcaster',
        namespace=drone_ns,
        parameters=[{'use_sim_time': True}],
        arguments=[
            '0.3', '0', '0.01', '0', '0', '0',
            f'{drone_ns}/base_link',
            f'{drone_ns}/lidar_sensor_link'
        ]
    )

    lidar_link_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='drone_lidar_link_bridge',
        namespace=drone_ns,
        parameters=[{'use_sim_time': True}],
        arguments=[
            '0', '0', '0', '0', '0', '0',
            f'{drone_ns}/lidar_sensor_link',
            'my_lidar_camera_drone_0/lidar_sensor_link/lidar_3d'
        ]
    )

    d_cam_to_rgb = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='drone_camera_to_imx214',
        parameters=[{'use_sim_time': True}],
        arguments=['0', '0', '0', '0', '0', '0',
                   f'{drone_ns}/camera_link',
                   'my_lidar_camera_drone_0/camera_link/IMX214']
    )

    odom_tf_publisher = Node(
        package='multi_drone_slam',
        executable='gz_link_pose_to_tf',
        name='drone_gz_link_pose_to_tf',
        namespace=drone_ns,
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'odom_topic': f'/{drone_ns}/local_position/odom',
            'parent_frame': f'{drone_ns}/odom',
            'child_frame':  f'{drone_ns}/base_link',
        }]
    )

    return LaunchDescription([
        gz_ros_bridge,
        TimerAction(period=8.0, actions=[static_map_to_drone_map]),
        TimerAction(period=8.0, actions=[mavros]),
        TimerAction(period=8.0, actions=[static_tf_publisher]),
        TimerAction(period=8.0, actions=[imu_tf_publisher]),
        TimerAction(period=8.0, actions=[lidar_tf_publisher]),
        TimerAction(period=8.0, actions=[lidar_link_bridge]),
        TimerAction(period=8.0, actions=[d_cam_to_rgb]),
        TimerAction(period=8.0, actions=[odom_tf_publisher]),
    ])
