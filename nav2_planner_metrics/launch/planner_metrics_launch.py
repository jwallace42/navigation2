import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # current_dir = get_package_share_directory('nav2_planner_metrics')

    #declare launch arguments 
    # declare_map_yaml_cmd = DeclareLaunchArgument(
    #     'map',
    #     default_value=os.path.join(
    #         current_dir, 'maps', 'turtlebot3_world.yaml'),
    #     description='Full path to map file to load')

    config = os.path.join(
        get_package_share_directory('nav2_planner_metrics'),
        'params',
        'planner_metric_params.yaml'
        )

    lifecycle_nodes = ['map_server']

    # start_planner_server = Node(
    #         package='nav2_planner',
    #         executable='planner_server',
    #         name='planner_server',
    #         output='screen',
    #         parameters=[config]
    # )

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[config]
        ),

        Node(
            package = 'tf2_ros',
            executable = 'static_transform_publisher',
            output = 'screen',
            arguments = ["0", "0", "0", "0", "0", "0", "odom", "map"]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': lifecycle_nodes}])

    ])