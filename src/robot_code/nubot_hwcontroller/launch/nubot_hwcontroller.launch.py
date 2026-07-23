"""为队伍中的每台机器人启动一个 ROS 2 仿真硬件控制器。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch_setup(context):
    """根据 launch 参数动态创建多个 nubot_hwcontroller_node。

    OpaqueFunction 允许先读取 team_size，再按机器人编号生成 nubot1、nubot2...
    等多个节点实例，每个节点负责自己命名空间下的 ActionCmd -> VelCmd。
    """
    team_prefix = LaunchConfiguration('team_prefix').perform(context)
    team_size = int(LaunchConfiguration('team_size').perform(context))
    nodes = []

    for robot_id in range(1, team_size + 1):
        robot_name = f'{team_prefix}{robot_id}'
        nodes.append(
            Node(
                package='nubot_hwcontroller',
                executable='nubot_hwcontroller_node',
                name=f'nubot_hwcontroller_{robot_id}',
                output='screen',
                parameters=[{
                    'robot_name': robot_name,
                    'team_prefix': team_prefix,
                    'team_info': ParameterValue(
                        LaunchConfiguration('team_info'), value_type=bool
                    ),
                    'control_period': ParameterValue(
                        LaunchConfiguration('control_period'), value_type=float
                    ),
                    'command_timeout': ParameterValue(
                        LaunchConfiguration('command_timeout'), value_type=float
                    ),
                    'use_sim_time': ParameterValue(
                        LaunchConfiguration('use_sim_time'), value_type=bool
                    ),
                }],
            )
        )
    return nodes


def generate_launch_description():
    """生成硬件控制器 launch 描述。

    参数：
    team_prefix: 队伍前缀，例如 nubot。
    team_size: 启动控制器的机器人数量。
    team_info: 传给节点的队伍标识，用于匹配红牌/恢复消息。
    control_period: 控制循环周期，单位秒。
    command_timeout: ActionCmd 超时时间，单位秒。
    use_sim_time: 是否使用仿真时钟。
    """
    return LaunchDescription([
        DeclareLaunchArgument('team_prefix', default_value='nubot'),
        DeclareLaunchArgument('team_size', default_value='5'),
        DeclareLaunchArgument('team_info', default_value='false'),
        DeclareLaunchArgument('control_period', default_value='0.005'),
        DeclareLaunchArgument('command_timeout', default_value='0.25'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=_launch_setup),
    ])
