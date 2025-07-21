from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_ros_bridge',
        parameters=[{
            'config_file': '/home/frazergene/drone/drone_ws/install/'
                            'multi_drone_slam/share/multi_drone_slam/'
                            'config/bridge_config.yaml'
        }],
        output='screen'
    )

    mavros1 = Node(
        package='mavros',
        executable='mavros_node',
        output='screen',
        arguments=[
            '--ros-args',
            '-p', 'fcu_url:=udp://@127.0.0.1:14580',
            '-r', '__ns:=/drone1'
        ]
    )

    mavros2 = Node(
        package='mavros',
        executable='mavros_node',
        output='screen',
        arguments=[
            '--ros-args',
            '-p', 'fcu_url:=udp://@127.0.0.1:14581',
            '-r', '__ns:=/drone2'
        ]
    )

    return LaunchDescription([
        bridge,
        TimerAction(period=2.0, actions=[mavros1]),
        TimerAction(period=4.0, actions=[mavros2])
    ])