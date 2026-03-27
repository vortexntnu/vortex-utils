from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('vortex_utility_nodes'),
        'config',
        'euler_odometry_publisher.yaml'
    )

    return LaunchDescription([
        Node(
            package='vortex_utility_nodes',
            executable='euler_odometry_publisher_node',
            name='euler_odometry_publisher_node',
            parameters=[config],
            output='screen',
        ),
    ])
