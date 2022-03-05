
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    bringup_dir = get_package_share_directory('nav2_bringup')
    # namespace = LaunchConfiguration('namespace')
    nav2_params_file = os.path.join(bringup_dir, 'params', 'nav2_params.yaml')

    # param_substitutions = { 'use_sim_time': True, 'autostart': True}

    # configured_params = RewrittenYaml(
    #     source_file=nav2_params_file,
    #     root_key=namespace,
    #     param_rewrites=param_substitutions,
    #     convert_types=True)

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    lifecycle_nodes = ['nav2_planner']

    return LaunchDescription(
        [
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                prefix=['xterm -e gdb -ex run --args'],
                output='screen',
                parameters=[nav2_params_file],
                remappings=remappings
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{'use_sim_time': True},
                            {'autostart': True},
                            {'node_names': lifecycle_nodes}]
            )
        ]
    )

