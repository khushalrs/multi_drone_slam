#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, GroupAction, DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    # --- 1) point Gazebo at your local models folder ---
    pkg_share = get_package_share_directory('multi_drone_slam')
    gz_models = os.path.join(pkg_share, 'models')
    os.environ['GZ_SIM_MODEL_PATH'] = gz_models
    os.environ['GZ_SIM_RESOURCE_PATH'] = gz_models

    # --- PX4 & MAVROS settings ---
    px4_root = os.path.expanduser('~/drone/PX4-Autopilot')
    px4_bin = os.path.join(px4_root, 'build/px4_sitl_default/bin/px4')

    ld = LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='True')
    ])

    # --- spawn the building, after a short delay ---
    building = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(gz_models, 'terrain', 'cylinder', 'model.sdf'),
            '-name', 'cylinder',
            '-x', '50', '-y', '0', '-z', '0',
            '-R', '0',  '-P', '0', '-Y', '0',
        ],
        output='screen'
    )
    ld.add_action(TimerAction(period=2.0, actions=[building]))

    # --- spawn single drone ---
    # PX4 SITL
    px4_1 = GroupAction([
        PushRosNamespace('drone1'),
        ExecuteProcess(
            cmd=[px4_bin, '-i', '0'],
            additional_env={
                'MAV_SYS_ID': '1',
                'PX4_GZ_MODEL_POSE': '0,0,0.07,0,0,0',
                'PX4_SIM_MODEL': 'gz_x500_depth_mono',
            },
            cwd=px4_root,
            output='screen',
            name='px4_0'
        )
    ])

    px4_2 = GroupAction([
        PushRosNamespace('drone2'),
        ExecuteProcess(
            cmd=[px4_bin, '-i', '1'],
            additional_env={
                'MAV_SYS_ID': '2',
                'PX4_GZ_MODEL_POSE': '-3,0,0.1,0,0,0',
                'PX4_SIM_MODEL': 'gz_x500_depth_mono',
            },
            cwd=px4_root,
            output='screen',
            name='px4_1'
        )
    ])

    ld.add_action(TimerAction(period=5.0, actions=[px4_1]))
    ld.add_action(TimerAction(period=18.0, actions=[px4_2]))

    return ld