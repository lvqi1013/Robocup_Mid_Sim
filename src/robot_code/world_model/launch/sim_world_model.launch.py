from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch_setup(context):
    team_prefix = LaunchConfiguration('team_prefix').perform(context)
    team_size = int(LaunchConfiguration('team_size').perform(context))
    nodes = []

    for robot_id in range(1, team_size + 1):
        robot_name = f'{team_prefix}{robot_id}'
        nodes.append(
            Node(
                package='world_model',
                executable='sim_world_model',
                name=f'world_model_{robot_id}',
                output='screen',
                parameters=[{
                    'robot_name': robot_name,
                    'team_prefix': team_prefix,
                    'team_size': team_size,
                    'update_period': ParameterValue(
                        LaunchConfiguration('update_period'),
                        value_type=float,
                    ),
                }],
            )
        )
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('team_prefix', default_value='nubot'),
        DeclareLaunchArgument('team_size', default_value='5'),
        DeclareLaunchArgument('update_period', default_value='0.015'),
        OpaqueFunction(function=_launch_setup),
    ])
