import os

from ament_index_python.packages import get_package_share_directory
from auv_setup.launch_arg_common import (
    declare_drone_and_namespace_args,
    resolve_drone_and_namespace,
)
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    _, namespace = resolve_drone_and_namespace(context)

    params = os.path.join(
        get_package_share_directory('vortex_utility_nodes'),
        'config',
        'rpy_publisher.yaml',
    )

    node = Node(
        package='vortex_utility_nodes',
        executable='rpy_publisher_node',
        name='rpy_publisher_node',
        namespace=namespace,
        parameters=[params],
        output='screen',
    )

    return [node]


def generate_launch_description():
    return LaunchDescription(
        declare_drone_and_namespace_args() + [OpaqueFunction(function=launch_setup)]
    )
