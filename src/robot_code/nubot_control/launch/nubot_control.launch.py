"""为一支队伍启动每台机器人的 nubot_control 节点."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch_setup(context):
    team_prefix = LaunchConfiguration('team_prefix').perform(context)
    team_size = int(LaunchConfiguration('team_size').perform(context))
    nodes = []
    for agent_id in range(1, team_size + 1):
        nodes.append(
            Node(
                package='nubot_control',
                executable='nubot_control_node',
                name=f'nubot_control_{agent_id}',
                output='screen',
                parameters=[{
                    'robot_name': f'{team_prefix}{agent_id}',
                    'team_prefix': team_prefix,
                    'team_size': team_size,
                    'control_period': ParameterValue(
                        LaunchConfiguration('control_period'), value_type=float
                    ),
                    'world_model_timeout': ParameterValue(
                        LaunchConfiguration('world_model_timeout'),
                        value_type=float,
                    ),
                    'use_sim_time': ParameterValue(
                        LaunchConfiguration('use_sim_time'), value_type=bool
                    ),
                }],
            )
        )
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('team_prefix', default_value='nubot'),
        DeclareLaunchArgument('team_size', default_value='5'),
        DeclareLaunchArgument('control_period', default_value='0.015'),
        DeclareLaunchArgument('world_model_timeout', default_value='0.25'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=_launch_setup),
    ])
