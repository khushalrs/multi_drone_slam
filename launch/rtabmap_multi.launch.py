import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, TimerAction, IncludeLaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from datetime import datetime

'''def generate_launch_description():
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
                    # 'wait_for_transform': '0.5',
                    # 'odom_sensor_sync': 'True'
                }.items()
            )
        ]
    )

    return LaunchDescription([
        Node(
            package='md_swarm_slam', executable='pose_to_tf', name='pose_to_tf_r2',
            parameters=[{'odom_frame':'drone1/odom'},{'base_frame':'drone1/base_link'}],
            namespace='drone1'),
        # Node(
        #     package='md_swarm_slam', executable='pose_to_tf', name='pose_to_tf_r2',
        #     parameters=[{'odom_frame':'drone2/odom'},{'base_frame':'drone2/base_link'}],
        #     namespace='drone2'),
        TimerAction(period=3.0, actions=[rtabmap1]),
        # TimerAction(period=6.0, actions=[rtabmap2])
    ])'''

def rgbd_pipeline_with_imu(ns: str, odom_topic: str, imu_topic: str, run_ts: str):
    return GroupAction([
        PushRosNamespace(ns),
        Node(
            package='multi_drone_slam',
            executable='imu_reframe',
            name='imu_reframe',
            output='screen',
            parameters=[{
                'in_topic':  f'/{ns}/imu/data',
                'out_topic': f'/{ns}/imu/data_fixed',
                'frame_id':  f'{ns}/base_link',
            }]
        ),

        # Pack RGB + Depth + Info into /<ns>/rgbd_image
        Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'approx_sync': True,
                'approx_sync_max_interval': 0.40,
                'qos_image': 2,
                'qos_camera_info': 2,
                'sync_queue_size': 400,
            }],
            remappings=[
                ('rgb/image',        f'/{ns}/rgb_camera'),
                ('depth/image',      f'/{ns}/depth_camera'),
                ('rgb/camera_info',  f'/{ns}/camera_info'),
                ('rgbd_image',       f'/{ns}/rgbd_image'),
            ],
        ),

        # Visual odometry, now consuming IMU
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'frame_id':      f'{ns}/base_link',
                'odom_frame_id': f'{ns}/odom',
                'publish_tf':    False,
                'subscribe_depth': False,
                'subscribe_rgbd':  True,
                'subscribe_imu':   True,
                'wait_imu_to_init': False,

                'sync_queue_size': 100,
                'approx_sync': True,
                'approx_sync_max_interval': 0.4,
                'wait_for_transform': 0.2,
                'qos_image': 2,
                'qos_camera_info': 2,

                # Robustness knobs
                'always_process_most_recent_frame': False,
                'Odom/ResetCountdown': '3',
                'Odom/GuessMotion': 'true',
                'Odom/GuessSmoothing': 'true',
                'Odom/MinInliers': '10',
                'Vis/MinInliers': '10',
                'Kp/MaxFeatures': '2000',
                'Kp/DetectorStrategy': '6',
                'GFTT/QualityLevel': '0.001',
                'GFTT/MinDistance': '2',
                'ORB/ScaleFactor': '1.2',
                'ORB/NLevels': '8',

                'RGBD/MinDepth': 0.2,
                'RGBD/MaxDepth': 8.0,
                'RGBD/DepthResolution': 2,
            }],
            remappings=[
                ('rgbd_image', f'/{ns}/rgbd_image'),
                ('odom', f'/{ns}/rgbd_odom'),
                ('imu', imu_topic),                 # <— wire in your IMU
            ],
        ),

        # RTAB-Map SLAM, also consuming IMU (for gravity/orientation priors)
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            output='screen',
            arguments=['--delete_db_on_start'],
            parameters=[{
                'use_sim_time': True,
                'frame_id':      f'{ns}/base_link',
                'odom_frame_id': f'{ns}/odom',
                'map_frame_id':  f'{ns}/map',
                'publish_tf':    True,
                'database_path': f'~/drone/drone_ws/src/md_swarm_slam/resource/{ns}_rtabmap_{run_ts}.db',

                'subscribe_depth': False,
                'subscribe_rgb':  False,
                'subscribe_rgbd': True,
                'subscribe_imu':  True,

                'topic_queue_size': 50,
                'sync_queue_size': 200,
                'qos_image': 2,
                'qos_camera_info': 2,
                'qos_odom': 2,

                'approx_sync': True,
                'approx_sync_max_interval': 0.4,

                'Odom/MinInliers': '10',
                'Vis/MinInliers': '10',
                'Kp/MaxFeatures': '2000',
                'Kp/DetectorStrategy': '6',
                'RGBD/MinDepth': '0.2',
                'RGBD/MaxDepth': '8.0',

                'RGBD/AngularUpdate': '0.05',
                'RGBD/LinearUpdate': '0.05',
                'RGBD/OptimizeFromGraphEnd': 'false',
                'Grid/RangeMin': '0.15',
                'Grid/RangeMax': '8.0',

                # Optional: lean more on gravity to stabilize roll/pitch
                'Mem/UseOdomGravity': 'true',
            }],
            remappings=[
                ('odom',        f'/{ns}/rgbd_odom'),
                ('rgbd_image',  f'/{ns}/rgbd_image'),
                ('imu',         imu_topic),        # <— IMU into RTAB-Map too
            ],
        ),
    ])


def generate_launch_description():
    run_ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    return LaunchDescription([
        # Keep your TF publisher bridges as before
        Node(
            package='md_swarm_slam', executable='pose_to_tf', name='pose_to_tf_r2',
            parameters=[{'odom_frame':'drone1/odom'},{'base_frame':'drone1/base_link'}],
            namespace='drone1'),
        # Node(
        #     package='md_swarm_slam', executable='pose_to_tf', name='pose_to_tf_r2',
        #     parameters=[{'odom_frame':'drone2/odom'},{'base_frame':'drone2/base_link'}],
        #     namespace='drone2'),

        # Pipelines with explicit IMU wiring
        rgbd_pipeline_with_imu('drone1', '/drone1/local_position/odom', '/drone1/imu/data_fixed', run_ts),
        # rgbd_pipeline_with_imu('drone2', '/drone2/local_position/odom', '/drone2/imu/data', run_ts),
    ])
