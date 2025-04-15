#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """Generate launch description for drone visualization."""
    
    # Get the package share directory
    pkg_share = get_package_share_directory('multi_drone_slam')
    
    # Path to RViz config file
    rviz_config = os.path.join(pkg_share, 'config', 'drone_viz.rviz')
    
    # Launch RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    
    # Launch pose visualizer
    pose_visualizer = Node(
        package='multi_drone_slam',
        executable='pose_visualizer',
        name='pose_visualizer',
        output='screen'
    )

    return LaunchDescription([
        pose_visualizer,
        rviz_node
    ]) 