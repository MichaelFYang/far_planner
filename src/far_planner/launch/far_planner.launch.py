import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PythonExpression
import launch_ros

def generate_launch_description():
    return LaunchDescription([
        SetParameter(name='use_sim_time', value='false'),
        DeclareLaunchArgument('config', default_value='default'),

        Node(
            package='far_planner',
            executable='far_planner',
            name='far_planner',
            output='screen',
            parameters=[
                PythonExpression([
                '"', 
                get_package_share_directory('far_planner'), 
                '/config/', 
                LaunchConfiguration('config'), 
                '.yaml"'])
            ],
            remappings=[
                ('/odom_world', '/odom'),
                ('/terrain_cloud', '/terrain_map_ext'),
                ('/scan_cloud', '/terrain_map'),
                ('/terrain_local_cloud', '/registered_scan')
            ]
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='far_rviz',
            arguments=['-d', 
                PythonExpression([
                '"', 
                get_package_share_directory('far_planner'), 
                '/rviz/far_planner.rviz"'])  # 直接指定文件名
            ],
            respawn=False,
        ),

        # Including another launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('graph_decoder'), '/launch/decoder.launch'])
        )
    ])
