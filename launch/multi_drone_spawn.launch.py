'''from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """Generate launch description for terrain mapping with camera bridge."""
    
    # Get the package share directory
    pkg_share = get_package_share_directory('multi_drone_slam')
    
    # Get paths
    model_path = os.path.join(pkg_share, 'models')
    # config_path = os.path.join(pkg_share, 'config')
    
    # # Set Gazebo model and resource paths
    gz_model_path = os.path.join(pkg_share, 'models')
    if 'GZ_SIM_MODEL_PATH' in os.environ:
        os.environ['GZ_SIM_MODEL_PATH'] += os.pathsep + gz_model_path
    else:
        os.environ['GZ_SIM_MODEL_PATH'] = gz_model_path

    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] += os.pathsep + gz_model_path
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = gz_model_path

    # Set initial drone pose (x y z roll pitch yaw)
    os.environ['PX4_GZ_MODEL_POSE'] = '0,0,0.1,0,0,0'

    px4_autopilot_path = LaunchConfiguration('px4_autopilot_path')

    num_drones = 3

    # Commands to spawn drones and bridges
    spawn_drones = []
    camera_bridges = []

    for i in range(num_drones):
        namespace = f"x500_gimbal_{i}"

        x = i * -3  # Each drone is spaced 2 meters apart along the x-axis 
        spawn_drones.append(
            TimerAction(
                period=5.0 * i,  # Delay to avoid collisions during spawning
                actions=[
                    ExecuteProcess(
                        cmd=['./build/px4_sitl_default/bin/px4', 
                            '-i', f'{i}'
                        ],
                        additional_env={
                            'PX4_SYS_AUTOSTART': '4001',
                            'PX4_GZ_MODEL_POSE': f'{x},0,0.1,0,0,0',
                            'PX4_SIM_MODEL': 'gz_x500_gimbal',
                            'PX4_UXRCE_DDS_NS' : f'px4_{i}',
                            'UXRCE_DDS_KEY' : f'{i+1}',
                        },
                        cwd=px4_autopilot_path,
                        output='screen',
                    )
                ]
            )
        )

        # Spawn PX4 SITL instance for the drone
        camera_bridges.append(
            TimerAction(
                period=7.0 * i,  # Delay to avoid collisions during spawning
                actions=[
                    Node(
                        package='ros_gz_bridge',
                        executable='parameter_bridge',
                        name=f'bridge_{namespace}',
                        parameters=[{
                            'use_sim_time': True,
                        }],
                        arguments=[
                            # Camera topics (one-way from Gazebo to ROS)
                            f'/world/default/model/{namespace}/camera@sensor_msgs/msg/Image[gz.msgs.Image',
                            f'/world/default/model/{namespace}/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                            # PX4 odometry (one-way from Gazebo to ROS)
                            f'/world/default/model/{namespace}/odometry_with_covariance@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                            # Clock (one-way from Gazebo to ROS)
                            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                        ],
                        remappings=[
                            (f'/model/{namespace}/camera', f'/{namespace}/camera'),
                            (f'/model/{namespace}/camera_info', f'/{namespace}/camera_info'),
                            (f'/model/{namespace}/odometry_with_covariance', f'/{namespace}/vehicle_odometry'),
                        ],
                        output='screen'
                    )
                ]
            )
        )

    spawn_building = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(model_path, 'terrain', 'skyscrapper', 'model.sdf'),
            '-name', 'building',
            '-x', '50',
            '-y', '0',
            '-z', '0',     # 5 meters below ground level
            '-R', '0', # Roll (90 degrees)
            '-P', '0',      # Pitch
            '-Y', '0'  # Yaw (90 degrees counterclockwise)
        ],
        output='screen'
    )

    delayed_building = TimerAction(
        period=2.0,  # 2 second delay
        actions=[spawn_building]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'px4_autopilot_path',
            default_value='/home/frazergene/drone/PX4-Autopilot',
            description='Path to PX4-Autopilot directory'
        ),
        delayed_building,
        *spawn_drones,
        *camera_bridges,
    ])


    # return LaunchDescription([
    #     DeclareLaunchArgument(
    #         'use_sim_time',
    #         default_value='True',
    #         description='Use simulation (Gazebo) clock if true'),
    #     DeclareLaunchArgument(
    #         'px4_autopilot_path',
    #         default_value='/home/frazergene/drone/PX4-Autopilot',
    #         description='Path to PX4-Autopilot directory'),
    #     px4_sitl,
    #     # delayed_terrain,
    #     TimerAction(
    #     period=2.0,  # 2 second delay
    #     actions=[spawn_building]),
    #     TimerAction(
    #     period=2.5,  # 5 second delay
    #     actions=[bridge])
    # ])
'''

# Three drones work and can be flown
'''
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, GroupAction
from launch_ros.actions import Node
import os

def generate_launch_description():
    # PX4 root directory
    px4_root = os.path.expanduser('~/drone/PX4-Autopilot')

    # Path to the pre-built PX4 binary
    px4_bin = os.path.join(px4_root, 'build/px4_sitl_default/bin/px4')

    # Number of drones
    num_drones = 3

    # Delay settings
    px4_startup_delay = 0.0    # PX4 starts immediately per drone group
    mavros_startup_delay = 8.0 # Wait for PX4 to be up before MAVROS
    drone_startup_interval = 5.0  # Delay between drones

    # Helper to create PX4 process
    def create_px4_process(instance_id, x_offset):
        return ExecuteProcess(
            cmd=[
                px4_bin,
                '-i', str(instance_id),
            ],
            additional_env={
                'PX4_GZ_MODEL_POSE': f'{x_offset},0,0.1,0,0,0',
                'PX4_SIM_MODEL': 'gz_x500_gimbal',
            },
            cwd=px4_root,
            output='screen',
            name=f'px4_instance_{instance_id}'
        )

    # Helper to create MAVROS2 node
    def create_mavros_node(instance_id, system_id):
        return ExecuteProcess(
            cmd=[
                'ros2', 'run', 'mavros', 'mavros_node',
                '--ros-args',
                '-p', f'fcu_url:=udp://@127.0.0.1:{14580 + instance_id}',
                '-r', f'__ns:=/drone{instance_id}'
            ],
            output='screen',
            name=f'mavros_node_{instance_id}'
        )
    
    def create_bridge_node(instance_id, system_id):
        return Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name=f'bridge_drone{instance_id}',
                parameters=[{'use_sim_time': True}],
                arguments=[
                    # Camera topics (GZ → ROS)
                    f'/world/default/model/x500_gimbal_{instance_id}/camera@sensor_msgs/msg/Image[gz.msgs.Image',
                    f'/world/default/model/x500_gimbal_{instance_id}/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                    # PX4 odometry (GZ → ROS)
                    f'/world/default/model/x500_gimbal_{instance_id}/odometry_with_covariance@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                    # Shared clock (GZ → ROS)
                    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                ],
                remappings=[
                    (f'/model/x500_gimbal_{instance_id}/camera',         f'/my_drone{instance_id}/camera'),
                    (f'/model/x500_gimbal_{instance_id}/camera_info',    f'/my_drone{instance_id}/camera_info'),
                    (f'/model/x500_gimbal_{instance_id}/odometry_with_covariance', f'/my_drone{instance_id}/vehicle_odometry'),
                ],
                output='screen'
            )
    
    # Spawn building in Gazebo
    spawn_building = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(px4_root, 'models', 'terrain', 'skyscrapper', 'model.sdf'),
            '-name', 'building',
            '-x', '50',
            '-y', '0',
            '-z', '0',     # 5 meters below ground level
            '-R', '0', # Roll (90 degrees)
            '-P', '0',      # Pitch
            '-Y', '0'  # Yaw (90 degrees counterclockwise)
        ],
        output='screen'
    )
    delayed_building = TimerAction(
        period=2.0,  # 2 second delay
        actions=[spawn_building]
    )

    # Launch description object
    ld = LaunchDescription()

    ld.add_action(delayed_building)
    # === Drone instances ===
    for instance_id in range(num_drones):
        x_offset = instance_id * -3  # Space drones 5 meters apart along the x-axis

        # PX4 process
        px4_process = create_px4_process(instance_id, x_offset)

        # MAVROS process (delayed after PX4)
        mavros_process_delayed = TimerAction(
            period=mavros_startup_delay,
            actions=[create_mavros_node(instance_id, instance_id + 1)]
        )
        # Bridge process (delayed after PX4)
        bridge_process_delayed = TimerAction(
            period=mavros_startup_delay,
            actions=[create_bridge_node(instance_id, instance_id + 1)]
        )

        # Group PX4 and MAVROS for this drone
        drone_group = GroupAction([
            px4_process,
            mavros_process_delayed,
            bridge_process_delayed
        ])

        # Delay entire drone group startup
        delayed_drone_group = TimerAction(
            period=drone_startup_interval * instance_id,
            actions=[drone_group]
        )

        # Add to launch description
        ld.add_action(delayed_drone_group)

    return ld'''

#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, GroupAction, DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --- 1) point Gazebo at your local models folder ---
    pkg_share = get_package_share_directory('multi_drone_slam')
    gz_models = os.path.join(pkg_share, 'models')
    os.environ['GZ_SIM_MODEL_PATH']    = gz_models
    os.environ['GZ_SIM_RESOURCE_PATH'] = gz_models

    # --- PX4 & MAVROS settings ---
    px4_root = os.path.expanduser('~/drone/PX4-Autopilot')
    px4_bin  = os.path.join(px4_root, 'build/px4_sitl_default/bin/px4')
    num_drones = 3
    mavros_delay = 8.0
    spawn_interval = 5.0

    ld = LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='True')
    ])

    # --- spawn the building, after a short delay ---
    building = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(gz_models, 'terrain', 'skyscrapper', 'model.sdf'),
            '-name', 'building',
            '-x', '50', '-y', '0', '-z', '0',
            '-R', '0',  '-P', '0', '-Y', '0',
        ],
        output='screen'
    )
    ld.add_action(TimerAction(period=2.0, actions=[building]))

    # --- loop over each drone instance ---
    for i in range(num_drones):
        x_off = i * -3

        # 1) PX4 SITL
        px4 = ExecuteProcess(
            cmd=[px4_bin, '-i', str(i)],
            additional_env={
                'PX4_GZ_MODEL_POSE': f'{x_off},0,0.1,0,0,0',
                'PX4_SIM_MODEL': 'gz_x500_gimbal',
            },
            cwd=px4_root,
            output='screen',
            name=f'px4_{i}'
        )

        # 2) MAVROS (delayed so PX4 can finish booting)
        mavros = TimerAction(
            period=8.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'run', 'mavros', 'mavros_node',
                        '--ros-args',
                        '-p', f'fcu_url:=udp://@127.0.0.1:{14580 + i}',
                        '-r', f'__ns:=/drone{i}'
                    ],
                    output='screen',
                    name=f'mavros_{i}'
                )
            ]
        )

        # 3) gz‑ros2 bridge (with *corrected* remappings)
        bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                f'/world/default/model/x500_gimbal_{i}/link/camera_link/sensor/image@sensor_msgs/msg/Image@gz.msgs.Image',
                f'/world/default/model/x500_gimbal_{i}/link/camera_link/sensor/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
            ],
            remappings=[
                (f'/world/default/model/x500_gimbal_{i}/link/camera_link/sensor/image', f'/drone{i}/camera/image_raw'),
                (f'/world/default/model/x500_gimbal_{i}/link/camera_link/sensor/camera_info', f'/drone{i}/camera/camera_info')
            ],
            name=f'camera_bridge_{i}'
        )

        # group them and stagger startup
        group = GroupAction([px4, bridge, mavros])
        ld.add_action(TimerAction(period=12.0 * i, actions=[group]))

    return ld

