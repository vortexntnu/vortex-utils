import os

from ament_index_python.packages import get_package_share_directory
from auv_setup.launch_arg_common import (
    declare_drone_and_namespace_args,
    resolve_drone_and_namespace,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    _, namespace = resolve_drone_and_namespace(context)

    environment = LaunchConfiguration('environment').perform(context)
    env_params = os.path.join(
        get_package_share_directory('auv_setup'),
        'config',
        'environments',
        f'{environment}.yaml',
    )

    node = Node(
        package='vortex_utility_nodes',
        executable='pressure_to_depth_node',
        name='pressure_to_depth',
        namespace=namespace,
        parameters=[
            env_params,
            {
                'transform_to_base_link': LaunchConfiguration('transform_to_base_link'),
            },
        ],
        output='screen',
    )

    return [node]


def generate_launch_description():
    environment_arg = DeclareLaunchArgument(
        'environment',
        default_value='trondheim_freshwater',
        description='Environment config to load water density and gravity from.',
        choices=[
            'longbeach',
            'stonefish_sim',
            'trondheim_freshwater',
            'trondheim_saltwater',
        ],
    )
    transform_arg = DeclareLaunchArgument(
        'transform_to_base_link',
        default_value='false',
        description='If true, apply the static pressure_sensor_link->base_link TF to shift depth to base_link frame.',
    )

    return LaunchDescription(
        [environment_arg, transform_arg]
        + declare_drone_and_namespace_args()
        + [OpaqueFunction(function=launch_setup)]
    )
