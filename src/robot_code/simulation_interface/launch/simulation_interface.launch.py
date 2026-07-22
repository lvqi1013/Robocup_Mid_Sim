from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """生成仿真接口节点的 launch 描述。

    启动 strategy_aggregator、coach_bridge 和 dribble_status_server 三个节点，
    为仿真中的策略聚合、教练信息广播、带球编号服务转话题提供统一入口。
    """
    # Launch 参数：
    # team_prefix: 队伍前缀，例如 nubot，用于节点内部拼接话题名。
    # team_size: 队伍机器人数量。
    # publish_rate: strategy_aggregator 和 coach_bridge 的发布频率，单位 Hz。
    # strategy_timeout: 单机器人策略缓存超时时间，单位秒。
    # match_mode/match_type/test_mode: coach_bridge 初始 CoachInfo 状态。
    team_prefix = LaunchConfiguration('team_prefix')
    team_size = LaunchConfiguration('team_size')
    publish_rate = LaunchConfiguration('publish_rate')
    strategy_timeout = LaunchConfiguration('strategy_timeout')
    match_mode = LaunchConfiguration('match_mode')
    match_type = LaunchConfiguration('match_type')
    test_mode = LaunchConfiguration('test_mode')

    return LaunchDescription([
        DeclareLaunchArgument('team_prefix', default_value='nubot'),
        DeclareLaunchArgument('team_size', default_value='5'),
        DeclareLaunchArgument('publish_rate', default_value='30.0'),
        DeclareLaunchArgument('strategy_timeout', default_value='0.10'),
        DeclareLaunchArgument('match_mode', default_value='0'),
        DeclareLaunchArgument('match_type', default_value='0'),
        DeclareLaunchArgument('test_mode', default_value='0'),
        # 聚合各机器人单独发布的 StrategyInfo，输出队伍级 SimulationStrategy。
        Node(
            package='simulation_interface',
            executable='strategy_aggregator',
            name='strategy_pub_node',
            output='screen',
            parameters=[{
                'team_prefix': team_prefix,
                'team_size': ParameterValue(team_size, value_type=int),
                'publish_rate': ParameterValue(publish_rate, value_type=float),
                'timeout_sec': ParameterValue(strategy_timeout, value_type=float),
            }],
        ),
        # 发布 CoachInfo，并把最新机器人 world_model 转换为 coach 世界模型。
        Node(
            package='simulation_interface',
            executable='coach_bridge',
            name='coach_bridge',
            output='screen',
            parameters=[{
                'team_prefix': team_prefix,
                'team_size': ParameterValue(team_size, value_type=int),
                'publish_rate': ParameterValue(publish_rate, value_type=float),
                'match_mode': ParameterValue(match_mode, value_type=int),
                'match_type': ParameterValue(match_type, value_type=int),
                'test_mode': ParameterValue(test_mode, value_type=int),
            }],
        ),
        # 提供 /DribbleId 服务，并将当前持球编号发布到 /dribble_id。
        Node(
            package='simulation_interface',
            executable='dribble_status_server',
            name='dribble_status_server',
            output='screen',
        ),
    ])
