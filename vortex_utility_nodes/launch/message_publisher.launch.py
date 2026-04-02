import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('vortex_utility_nodes'),
        'config',
        'message_publisher.yaml',
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace prepended to the output topic and node name',
    )

    return LaunchDescription(
        [
            namespace_arg,
            Node(
                package='vortex_utility_nodes',
                executable='message_publisher_node',
                name='message_publisher_node',
                namespace=LaunchConfiguration('namespace'),
                parameters=[config],
                output='screen',
            ),
        ]
    )
