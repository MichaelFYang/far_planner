from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import yaml

def generate_launch_description():
    boundary_handler_path = get_package_share_directory('boundary_handler')
    config_path = boundary_handler_path + '/config/default.yaml'

    # Load the YAML file
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    return LaunchDescription([
        DeclareLaunchArgument('boundary_file', default_value='boundary.ply'),
        DeclareLaunchArgument('traj_file', default_value='trajectory.txt'),
        DeclareLaunchArgument('graph_file', default_value='boundary_graph.vgh'),

        Node(
            package='boundary_handler',
            executable='boundary_handler',
            name='boundary_handler',
            output='screen',
            parameters=[
                {'boundary_file': LaunchConfiguration('boundary_file')},
                {'traj_file': LaunchConfiguration('traj_file')},
                {'graph_file': LaunchConfiguration('graph_file')},
                {'folder_path': get_package_share_directory('far_planner') + '/data/'},
                config,
            ]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='boundary_rviz',
            arguments=['-d', get_package_share_directory('boundary_handler') + '/rviz/default.rviz'],
            respawn=False,
        ),
    ])
