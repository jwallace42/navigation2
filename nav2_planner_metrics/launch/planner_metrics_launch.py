from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen'
        ),
        Node(
            package = 'tf2_ros',
            executable = 'static_transform_publisher',
            output = 'screen',
            arguments = ["0", "0", "0", "0", "0", "0", "odom", "map"]
        )
    ])