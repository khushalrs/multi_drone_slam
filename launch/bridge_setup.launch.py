# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.actions import TimerAction

# def generate_launch_description():
#     bridge = Node(
#         package='ros_gz_bridge',
#         executable='parameter_bridge',
#         name='gz_ros_bridge',
#         parameters=[{
#             'config_file': '/home/frazergene/drone/drone_ws/install/'
#                             'multi_drone_slam/share/multi_drone_slam/'
#                             'config/bridge_config.yaml'
#         }],
#         output='screen'
#     )

    # mavros1 = Node(
    #     package='mavros',
    #     executable='mavros_node',
    #     output='screen',
    #     arguments=[
    #         '--ros-args',
    #         '-p', 'fcu_url:=udp://@127.0.0.1:14580',
    #         '-r', '__ns:=/drone1'
    #     ]
    # )

    # mavros2 = Node(
    #     package='mavros',
    #     executable='mavros_node',
    #     output='screen',
    #     arguments=[
    #         '--ros-args',
    #         '-p', 'fcu_url:=udp://@127.0.0.1:14581',
    #         '-r', '__ns:=/drone2'
    #     ]
    # )

#     # static world->map (identity)
#     world_to_map = Node(
#         package='tf2_ros', executable='static_transform_publisher',
#         arguments=['0','0','0','0','0','0','world','map'],
#         output='screen'
#     )

#     tf_d1 = Node(
#         package='multi_drone_slam',
#         executable='gz_link_pose_to_tf',
#         name='tf_drone1',
#         arguments=['drone1/base_link', '/drone1/gz/base_link_pose', 'map'],
#         output='screen'
#     )

#     tf_d2 = Node(
#         package='multi_drone_slam',
#         executable='gz_link_pose_to_tf',
#         name='tf_drone2',
#         arguments=['drone2/base_link', '/drone2/gz/base_link_pose', 'map'],
#         output='screen'
#     )

#     return LaunchDescription([
#         bridge,
#         TimerAction(period=1.0, actions=[tf_d1]),
#         TimerAction(period=1.5, actions=[tf_d2]),
#         TimerAction(period=2.0, actions=[world_to_map]),
#         TimerAction(period=3.0, actions=[mavros1]),
#         TimerAction(period=5.0, actions=[mavros2])
#     ])


import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generates the launch description for a multi-drone SLAM setup.

    This launch file starts the necessary components for two drones, including:
    - A Gazebo to ROS bridge for sensor data.
    - MAVROS for communication with the PX4 autopilot.
    - A static transform publisher for the camera's physical location on the drone.
    - RTAB-Map for simultaneous localization and mapping.
    """
    pkg_share = get_package_share_directory('multi_drone_slam')
    bridge_config_path = os.path.join(pkg_share, 'config', 'bridge_config.yaml')
    mavros_config_path = os.path.join(pkg_share, 'config', 'mavros_config.yaml')

    # --- SHARED NODES ACROSS ALL DRONES ---
    gz_ros_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_ros_bridge',
        parameters=[{'config_file': bridge_config_path}],
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
        ]
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
        ]
    )

    # --- DRONE 1 SETUP ---
    drone1_ns = 'drone1'
    # mavros1 = Node(
    #     package='mavros',
    #     executable='mavros_node',
    #     namespace=drone1_ns,
    #     output='screen',
    #     parameters=[
    #         mavros_config_path,
    #         {'fcu_url': 'udp://@127.0.0.1:14580'}
    #     ]
    # )

    static_tf_publisher1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_broadcaster',
        namespace=drone1_ns,
        arguments=[
            '0.12', '0.03', '0.242', '0', '0', '0',
            f'{drone1_ns}/base_link',
            f'{drone1_ns}/camera_link'
        ]
    )
    
    # This node will subscribe to MAVROS odometry and publish the odom->base_link transform
    # This is a crucial link that RTAB-Map needs.
    odom_tf_publisher1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_link_publisher',
        namespace=drone1_ns,
        # NOTE: This should be a dynamic transform publisher that reads from odometry.
        # For initial testing, a static one at the origin is okay.
        arguments=['0', '0', '0', '0', '0', '0', f'{drone1_ns}/odom', f'{drone1_ns}/base_link']
    )

    # rtabmap1 = Node(
    #     package='rtabmap_ros',
    #     executable='rtabmap',
    #     name='rtabmap',
    #     namespace=drone1_ns,
    #     output='screen',
    #     parameters=[{
    #         'frame_id': f'{drone1_ns}/base_link',
    #         'odom_frame_id': f'{drone1_ns}/odom',
    #         'map_frame_id': f'{drone1_ns}/map',
    #         'subscribe_depth': True,
    #         'subscribe_rgb': True,
    #         'subscribe_odom': True,
    #         'queue_size': 100,
    #     }],
    #     remappings=[
    #         ('rgb/image', 'rgb_camera'),
    #         ('depth/image', 'depth_camera'),
    #         ('rgb/camera_info', 'camera_info'),
    #         ('odom', 'mavros/local_position/odom')
    #     ]
    # )

    # --- DRONE 2 SETUP ---
    drone2_ns = 'drone2'
    # mavros2 = Node(
    #     package='mavros',
    #     executable='mavros_node',
    #     namespace=drone2_ns,
    #     output='screen',
    #     parameters=[
    #         mavros_config_path,
    #         {'fcu_url': 'udp://@127.0.0.1:14581'}
    #     ]
    # )

    static_tf_publisher2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_broadcaster',
        namespace=drone2_ns,
        arguments=[
            '0.12', '0.03', '0.242', '0', '0', '0',
            f'{drone2_ns}/base_link',
            f'{drone2_ns}/camera_link'
        ]
    )

    odom_tf_publisher2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_link_publisher',
        namespace=drone2_ns,
        arguments=['0', '0', '0', '0', '0', '0', f'{drone2_ns}/odom', f'{drone2_ns}/base_link']
    )

    # rtabmap2 = Node(
    #     package='rtabmap_ros',
    #     executable='rtabmap',
    #     name='rtabmap',
    #     namespace=drone2_ns,
    #     output='screen',
    #     parameters=[{
    #         'frame_id': f'{drone2_ns}/base_link',
    #         'odom_frame_id': f'{drone2_ns}/odom',
    #         'map_frame_id': f'{drone2_ns}/map',
    #         'subscribe_depth': True,
    #         'subscribe_rgb': True,
    #         'subscribe_odom': True,
    #         'queue_size': 100,
    #     }],
    #     remappings=[
    #         ('rgb/image', 'rgb_camera'),
    #         ('depth/image', 'depth_camera'),
    #         ('rgb/camera_info', 'camera_info'),
    #         ('odom', 'mavros/local_position/odom')
    #     ]
    # )

    return LaunchDescription([
        gz_ros_bridge,
        mavros1, static_tf_publisher1, odom_tf_publisher1, 
        # rtabmap1,
        mavros2, static_tf_publisher2, odom_tf_publisher2, # rtabmap2
    ])

