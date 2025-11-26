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
    bridge_config_path = os.path.join(pkg_share, 'config', 'single_bridge_config.yaml')
    mavros_config_path = os.path.join(pkg_share, 'config', 'mavros_config.yaml')

    # --- SHARED NODES ACROSS ALL DRONES ---
    gz_ros_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_ros_bridge',
        parameters=[{'config_file': bridge_config_path, 'use_sim_time': True}],
        output='screen'
    )

    mavros1 = Node(
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

    mavros2 = Node(
        package='mavros',
        executable='mavros_node',
        output='screen',
        arguments=[
            '--ros-args',
            '-p', 'fcu_url:=udp://:14541@127.0.0.1:14581',
            '-p', 'tgt_system:=2',
            '-p', 'fcu_protocol:=v2.0',
            '-r', '__ns:=/drone2'
        ],
        parameters=[{'use_sim_time': True}]
    )

    # --- Static transforms to connect each drone's map to a global 'map' frame ---
    # This allows visualizing both drones in RViz under a common frame.
    # RTAB-Map will publish the transform from its map_frame (e.g., 'drone1/map')
    # to its odom_frame (e.g., 'drone1/odom').

    # Drone 1 spawns at (0, 0, ...), so its map frame is aligned with the global map frame.
    static_map_to_drone1_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_drone1_map_publisher',
        parameters= [{'use_sim_time': True}],
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'drone1/map']
    )

    # Drone 2 spawns at (-3, 0, ...), so its map frame is offset from the global map frame.
    static_map_to_drone2_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_drone2_map_publisher',
        parameters= [{'use_sim_time': True}],
        arguments=['-3', '0', '0', '0', '0', '0', 'map', 'drone2/map']
    )

    # --- DRONE 1 SETUP ---
    drone1_ns = 'drone1'

    static_tf_publisher1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='drone1_base_to_camera_broadcaster',
        namespace=drone1_ns,
        parameters=[{'use_sim_time': True}],
        arguments=[
            '0.12', '0.03', '0.242', '0', '0', '0',
            f'{drone1_ns}/base_link',
            f'{drone1_ns}/camera_link'
        ]
    )

    d1_cam_to_rgb = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='drone1_camera_to_imx214',
        parameters=[{'use_sim_time': True}],
        arguments=['0', '0', '0', '0', '0', '0',
                   f'{drone1_ns}/camera_link',
                   'x500_depth_mono_0/camera_link/IMX214']
    )

    d1_cam_to_stereo = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='drone1_camera_to_stereo',
        parameters=[{'use_sim_time': True}],
        arguments=['0', '0', '0', '0', '0', '0',
                   f'{drone1_ns}/camera_link',
                   'x500_depth_mono_0/camera_link/StereoOV7251']
    )

    odom_tf_publisher1 = Node(
        package='multi_drone_slam',
        executable='gz_link_pose_to_tf',
        name='drone1_gz_link_pose_to_tf',
        namespace=drone1_ns,
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'odom_topic': f'/{drone1_ns}/local_position/odom',
            'parent_frame': f'{drone1_ns}/odom',
            'child_frame':  f'{drone1_ns}/base_link',
        }]
    )

    # --- DRONE 2 SETUP ---
    drone2_ns = 'drone2'

    static_tf_publisher2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='drone2_base_to_camera_broadcaster',
        namespace=drone2_ns,
        parameters=[{'use_sim_time': True}],
        arguments=[
            '0.12', '0.03', '0.242', '0', '0', '0',
            f'{drone2_ns}/base_link',
            f'{drone2_ns}/camera_link'
        ]
    )

    d2_cam_to_rgb = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='drone2_camera_to_imx214',
        parameters=[{'use_sim_time': True}],
        arguments=['0', '0', '0', '0', '0', '0',
                   f'{drone2_ns}/camera_link',
                   'x500_depth_mono_1/camera_link/IMX214']
    )

    d2_cam_to_stereo = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='drone2_camera_to_stereo',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '0', '0', '0', '0', '0', '0',
                   f'{drone2_ns}/camera_link',
                   'x500_depth_mono_1/camera_link/StereoOV7251']
    )

    odom_tf_publisher2 = Node(
        package='multi_drone_slam',
        executable='gz_link_pose_to_tf',
        name='drone2_gz_link_pose_to_tf',
        namespace=drone2_ns,
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'odom_topic': f'/{drone2_ns}/local_position/odom',
            'parent_frame': f'{drone2_ns}/odom',
            'child_frame':  f'{drone2_ns}/base_link'
        }]
    )

    return LaunchDescription([
        gz_ros_bridge,
        TimerAction(period=8.0, actions=[static_map_to_drone1_map]),
        # TimerAction(period=8.0, actions=[static_map_to_drone2_map]),
        TimerAction(period=8.0, actions=[mavros1]),
        TimerAction(period=8.0, actions=[static_tf_publisher1]),
        TimerAction(period=8.0, actions=[d1_cam_to_rgb]),
        TimerAction(period=8.0, actions=[d1_cam_to_stereo]),
        TimerAction(period=8.0, actions=[odom_tf_publisher1]),
        # TimerAction(period=8.0, actions=[mavros2]),
        # TimerAction(period=8.0, actions=[static_tf_publisher2]),
        # TimerAction(period=8.0, actions=[d2_cam_to_rgb]),
        # TimerAction(period=8.0, actions=[d2_cam_to_stereo]),
        # TimerAction(period=8.0, actions=[odom_tf_publisher2])
    ])
