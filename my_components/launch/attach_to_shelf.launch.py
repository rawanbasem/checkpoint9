import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    rviz_config_path = os.path.join(
        get_package_share_directory('my_components'),
        'rviz',
        'config.rviz'
    )

    # Component Container
    container = ComposableNodeContainer(
        name='my_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt', 
        composable_node_descriptions=[
            ComposableNode(
                package='my_components',
                plugin='my_components::PreApproach',
                name='pre_approach'
            ),
            ComposableNode(
                package='my_components',
                plugin='my_components::AttachServer',
                name='attach_server'
            ),
        ],
        output='screen',
    )

    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        container,
        rviz_node
    ])