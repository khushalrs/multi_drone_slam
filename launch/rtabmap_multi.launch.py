import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    """
    Launches RTAB-Map nodes for two separate drones.
    """
    rtabmap_launch_dir = os.path.join(get_package_share_directory('rtabmap_launch'), 'launch')

    # --- DRONE 1 RTAB-MAP ---
    drone1_ns = 'drone1'
    rtabmap1 = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(rtabmap_launch_dir, 'rtabmap.launch.py')
                ),
                launch_arguments={
                    'namespace': drone1_ns,
                    'rviz': 'True',
                    'rtabmapviz': 'False',
                    'use_sim_time': 'True',
                    'publish_tf': 'False',
                    'frame_id': f'{drone1_ns}/base_link',
                    'odom_frame_id': f'{drone1_ns}/odom',
                    'map_frame_id': f'{drone1_ns}/map',
                    'subscribe_depth': 'True',
                    'subscribe_scan': 'False',
                    'approx_sync': 'True',
                    'approx_sync_max_interval': '0.2', # Increase tolerance for sim time sync
                    'database_path': f'~/drone/rtabmap_simulations/rtabmap_{drone1_ns}.db',
                    # Remap topics directly
                    'rgb_topic': f'/{drone1_ns}/rgb_camera',
                    'depth_topic': f'/{drone1_ns}/depth_camera',
                    'camera_info_topic': f'/{drone1_ns}/camera_info',
                    'odom_topic': f'/{drone1_ns}/odometry/in',
                    # 'wait_for_transform': '0.5',
                    # 'odom_sensor_sync': 'True'
                }.items()
            )
        ]
    )

    # --- DRONE 2 RTAB-MAP ---
    drone2_ns = 'drone2'
    rtabmap2 = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(rtabmap_launch_dir, 'rtabmap.launch.py')
                ),
                launch_arguments={
                    'namespace': drone2_ns,
                    'rviz': 'True',
                    'rtabmapviz': 'False',
                    'use_sim_time': 'True',
                    'publish_tf': 'False',
                    'frame_id': f'{drone2_ns}/base_link',
                    'odom_frame_id': f'{drone2_ns}/odom',
                    'map_frame_id': f'{drone2_ns}/map',
                    'subscribe_depth': 'True',
                    'subscribe_scan': 'False',
                    'approx_sync': 'True',
                    'approx_sync_max_interval': '0.2', # Increase tolerance for sim time sync
                    'database_path': f'~/drone/rtabmap_simulations/rtabmap_{drone2_ns}.db',
                    # Remap topics directly
                    'rgb_topic': f'/{drone2_ns}/rgb_camera',
                    'depth_topic': f'/{drone2_ns}/depth_camera',
                    'camera_info_topic': f'/{drone2_ns}/camera_info',
                    'odom_topic': f'/{drone2_ns}/odometry/in',
                    # 'wait_for_transform': '0.5',
                    # 'odom_sensor_sync': 'True'
                }.items()
            )
        ]
    )

    return LaunchDescription([
        TimerAction(period=3.0, actions=[rtabmap1]),
        TimerAction(period=6.0, actions=[rtabmap2])
    ])
