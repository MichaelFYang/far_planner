from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import yaml

def generate_launch_description():
    graph_decoder_path = get_package_share_directory('graph_decoder')
    config_path = graph_decoder_path + '/config/default.yaml'

    # Load the YAML file
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    return LaunchDescription([
        DeclareLaunchArgument(
            'graph_topic', 
            default_value='/planner_nav_graph', 
            description='Graph topic to be used'
        ),

        Node(
            package='graph_decoder',
            executable='graph_decoder',
            name='graph_decoder',
            output='screen',
            parameters=[
                config
            ],
            remappings=[
                ('/planner_nav_graph', LaunchConfiguration('graph_topic'))
            ],
        ),

        # Uncomment the following if you want to include RViz in your launch
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='decodeRiz',
        #     arguments=['-d', FindPackageShare('graph_decoder') + '/rviz/decoder.rviz']
        # ),
    ])
