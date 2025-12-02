from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # --------------------
    # Launch arguments
    # --------------------
    ns = LaunchConfiguration('namespace')
    frame_id = LaunchConfiguration('frame_id')
    odom_frame_id = LaunchConfiguration('odom_frame_id')
    map_frame_id = LaunchConfiguration('map_frame_id')

    rgb_topic = LaunchConfiguration('rgb_topic')
    rgb_info_topic = LaunchConfiguration('rgb_info_topic')
    lidar_topic = LaunchConfiguration('lidar_topic')
    imu_topic = LaunchConfiguration('imu_topic')

    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='drone1',
        description='Robot namespace (e.g., drone1)'
    )

    declare_frame_id = DeclareLaunchArgument(
        'frame_id',
        default_value='drone1/base_link',
        description='Base frame of the robot'
    )

    declare_odom_frame_id = DeclareLaunchArgument(
        'odom_frame_id',
        default_value='drone1/odom',
        description='Odom frame'
    )

    declare_map_frame_id = DeclareLaunchArgument(
        'map_frame_id',
        default_value='drone1/map',
        description='Map frame used by RTAB-Map'
    )

    declare_rgb_topic = DeclareLaunchArgument(
        'rgb_topic',
        default_value='/drone1/rgb_camera',
        description='RGB image topic'
    )

    declare_rgb_info_topic = DeclareLaunchArgument(
        'rgb_info_topic',
        default_value='/drone1/camera_info',
        description='Camera info topic'
    )

    declare_lidar_topic = DeclareLaunchArgument(
        'lidar_topic',
        default_value='/drone1/lidar/points',
        description='3D LiDAR PointCloud2 topic'
    )

    declare_imu_topic = DeclareLaunchArgument(
        'imu_topic',
        default_value='/drone1/imu_fixed/data',
        description='IMU topic'
    )

    # --------------------
    # ICP ODOMETRY (LiDAR)
    # --------------------
    icp_odom_params = {
        # Frames
        'frame_id': frame_id,
        'odom_frame_id': odom_frame_id,

        # Use 3D pointcloud (scan_cloud) input
        'scan_cloud': True,

        # Typical lidar setup parameters can be tuned later:
        'scan_downsampling_step': 1,   # keep all points (every nth point)
        'scan_voxel_size': 0.1,        # small voxel (10cm) to reduce noise but keep structure
        'scan_range_min': 0.3,         # ignore very near junk, adapt to your sensor
        'scan_range_max': 30.0,        # or your lidar max range

        # if your lidar is effectively planar (2D), consider:
        # 'scan_cloud_is_2d': True,

        # ---- ICP core parameters (go into RTAB-Map core) ----
        # Allow larger motion between scans so it can still match
        'Icp/MaxCorrespondenceDistance': '2.0',  # meters, tune as needed
        'Icp/PointToPlane': 'false',             # try point-to-point first (more stable for noisy clouds)

        # QoS (since you’re in ROS2 + likely sensor_data publishers)
        'qos': 2,        
    }

    icp_odometry_node = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='icp_odometry',
        namespace=ns,
        output='screen',
        parameters=[icp_odom_params],
        remappings=[
            # LiDAR input
            ('scan_cloud', lidar_topic),

            # If you also want IMU for guess, icp_odometry can use it via TF / external odom.
            # You can add imu remap if you configure icp to use it:
            # ('imu', imu_topic),

            # Odometry output (in this namespace → <ns>/odom)
            ('odom', 'odom'),
        ]
    )

    # --------------------
    # RTAB-Map SLAM NODE
    # --------------------
    rtabmap_params = {
        # Frames
        'frame_id': frame_id,
        'odom_frame_id': odom_frame_id,
        'map_frame_id': map_frame_id,

        # Sensor subscriptions: RGB + LiDAR + IMU
        'subscribe_depth': False,
        'subscribe_rgb': True,
        'subscribe_scan': False,
        'subscribe_scan_cloud': True,
        'subscribe_stereo': False,
        'subscribe_imu': True,
        'subscribe_odom_info': False,

        # Sync
        'approx_sync': True,
        'queue_size': 10,

        # Useful RTAB-Map behavior (can tune)
        'RGBD/LinearUpdate': '0.05',
        'RGBD/AngularUpdate': '0.05',
        'RGBD/ProximityBySpace': 'true',
        'Reg/Strategy': '1',  # 1 = Visual + ICP
        'Mem/IncrementalMemory': 'true',
        'Mem/InitWMWithAllNodes': 'false',

        # Multi-sensor sync compensation (camera+lidar+imu)
        'Odom/ResetCountdown': '0',
        'Odom/GuessSmoothing': 'true',
        'Odom/GuessFrameId': odom_frame_id,
        'RGBD/OptimizeFromGraphEnd': 'false',
        'RGBD/ProximityByTime': 'false',
        'RGBD/ProximityBySpace': 'true',
        'RGBD/OptimizeMaxError': '3.0',
        'odom_sensor_sync': True,  # important when mixing lidar/camera/imu
        'qos': 2,                  # important for ROS2 sensor topics
    }

    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        namespace=ns,
        output='screen',
        parameters=[rtabmap_params],
        remappings=[
            # Camera
            ('rgb/image', rgb_topic),
            ('rgb/camera_info', rgb_info_topic),

            # LiDAR for mapping
            ('scan_cloud', lidar_topic),

            # Use ICP odom from icp_odometry node
            ('odom', 'odom'),

            # IMU
            ('imu', imu_topic),

            # Optional: remap outputs if you want per-drone topics
            # ('map', 'map'),
            # ('grid_map', 'grid_map'),
            # ('cloud_map', 'cloud_map'),
            # ('cloud_obstacles', 'cloud_obstacles'),
        ]
    )

    # --------------------
    # RTAB-Map Visualizer
    # --------------------
    # Launch in SAME namespace so topics/services line up (as recommended). 
    rtabmap_viz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        namespace=ns,
        output='screen',
        parameters=[{
            'frame_id': 'drone1/base_link',
            'odom_frame_id': 'drone1/odom',
            'map_frame_id': 'drone1/map',

            'subscribe_depth': False,
            'subscribe_rgb': True,
            'subscribe_scan_cloud': True,
            'subscribe_imu': True,

            # Match sensor QoS (if your camera/lidar are sensor_data / best effort)
            'qos_image': 2,
            'qos_camera_info': 2,
            'qos_scan': 2,
            'qos_odom': 2,
        }],
        remappings=[
            ('rgb/image', '/drone1/rgb_camera'),
            ('rgb/camera_info', '/drone1/camera_info'),
            ('scan_cloud', '/drone1/lidar/points'),
            ('imu', '/drone1/imu_fixed/data'),
            ('odom', 'odom'),
        ],
    )

    return LaunchDescription([
        declare_namespace,
        declare_frame_id,
        declare_odom_frame_id,
        declare_map_frame_id,
        declare_rgb_topic,
        declare_rgb_info_topic,
        declare_lidar_topic,
        declare_imu_topic,
        icp_odometry_node,
        rtabmap_node,
        rtabmap_viz_node,
    ])
