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

    input_type_arg = DeclareLaunchArgument(
        'input_type',
        default_value='odometry',
        description='Input message type: odometry, waypoint, reference_filter, '
        'pose_stamped, or pose_with_covariance_stamped',
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace prepended to the output topic and node name',
    )

    return LaunchDescription(
        [
            input_type_arg,
            namespace_arg,
            Node(
                package='vortex_utility_nodes',
                executable='message_publisher_node',
                name='message_publisher_node',
                namespace=LaunchConfiguration('namespace'),
                parameters=[
                    config,
                    {'input_type': LaunchConfiguration('input_type')},
                ],
                output='screen',
            ),
        ]
    )
