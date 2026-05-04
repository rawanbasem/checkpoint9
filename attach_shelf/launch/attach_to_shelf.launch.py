import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare  launch arguments
    obstacle_arg = DeclareLaunchArgument('obstacle', default_value='0.4')
    degrees_arg = DeclareLaunchArgument('degrees', default_value='-90.0')
    final_approach_arg = DeclareLaunchArgument('final_approach', default_value='true')

    pkg_dir = get_package_share_directory('attach_shelf')
    rviz_config_file = os.path.join(pkg_dir, 'rviz', 'config.rviz')

    # Service Server
    server_node = Node(
        package='attach_shelf',
        executable='approach_service_server',
        name='approach_service_server',
        output='screen'
    )

    # Pre-Approach Client
    client_node = Node(
        package='attach_shelf',
        executable='pre_approach_v2_node',
        name='pre_approach_v2_node',
        output='screen',
        parameters=[{
            'obstacle': LaunchConfiguration('obstacle'),
            'degrees': LaunchConfiguration('degrees'),
            'final_approach': LaunchConfiguration('final_approach')
        }]
    )

    # RViz 
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        obstacle_arg,
        degrees_arg,
        final_approach_arg,
        server_node,
        client_node,
        rviz_node
    ])