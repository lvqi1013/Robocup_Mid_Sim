"""Launch the ROS 2 automatic referee for Gazebo Harmonic."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('cyan_prefix', default_value='nubot'),
        DeclareLaunchArgument('magenta_prefix', default_value='rival'),
        DeclareLaunchArgument('team_size', default_value='5'),
        DeclareLaunchArgument('start_team', default_value='-1'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('enforce_area_rules', default_value='true'),
        Node(
            package='auto_referee',
            executable='auto_referee_node',
            name='auto_referee',
            output='screen',
            parameters=[{
                'cyan_prefix': LaunchConfiguration('cyan_prefix'),
                'magenta_prefix': LaunchConfiguration('magenta_prefix'),
                'team_size': ParameterValue(
                    LaunchConfiguration('team_size'), value_type=int
                ),
                'start_team': ParameterValue(
                    LaunchConfiguration('start_team'), value_type=int
                ),
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool
                ),
                'enforce_area_rules': ParameterValue(
                    LaunchConfiguration('enforce_area_rules'), value_type=bool
                ),
            }],
        ),
    ])
