from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_name", default_value="nubot1"),
            DeclareLaunchArgument("team_prefix", default_value="nubot"),
            DeclareLaunchArgument("action_topic", default_value="/nubot1/nubotcontrol/actioncmd"),
            DeclareLaunchArgument("velcmd_topic", default_value="/nubot1/nubotcontrol/velcmd"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/nubot1/cmd_vel"),
            DeclareLaunchArgument("cmd_vel_override_topic", default_value="/nubot1/hwcontroller/cmd_vel_override"),
            DeclareLaunchArgument("dribble_enable_topic", default_value="/nubot1/dribble_enable"),
            DeclareLaunchArgument("shoot_power_topic", default_value="/nubot1/shoot_power"),
            DeclareLaunchArgument("ball_holding_topic", default_value="/nubot1/ball_is_holding"),
            DeclareLaunchArgument("redcard_topic", default_value="/nubot/redcard/chatter"),
            DeclareLaunchArgument("update_rate", default_value="100.0"),
            DeclareLaunchArgument("action_timeout", default_value="0.25"),
            DeclareLaunchArgument("position_gain", default_value="5.0"),
            DeclareLaunchArgument("rotation_gain", default_value="2.0"),
            DeclareLaunchArgument("linear_scale", default_value="0.01"),
            DeclareLaunchArgument("max_linear_speed", default_value="2.0"),
            DeclareLaunchArgument("max_angular_speed", default_value="6.0"),
            Node(
                package="nubot_hwcontroller",
                executable="hwcontroller_node",
                name=[LaunchConfiguration("robot_name"), "_hwcontroller"],
                output="screen",
                parameters=[
                    {
                        "robot_name": LaunchConfiguration("robot_name"),
                        "team_prefix": LaunchConfiguration("team_prefix"),
                        "action_topic": LaunchConfiguration("action_topic"),
                        "velcmd_topic": LaunchConfiguration("velcmd_topic"),
                        "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                        "cmd_vel_override_topic": LaunchConfiguration("cmd_vel_override_topic"),
                        "dribble_enable_topic": LaunchConfiguration("dribble_enable_topic"),
                        "shoot_power_topic": LaunchConfiguration("shoot_power_topic"),
                        "ball_holding_topic": LaunchConfiguration("ball_holding_topic"),
                        "redcard_topic": LaunchConfiguration("redcard_topic"),
                        "update_rate": LaunchConfiguration("update_rate"),
                        "action_timeout": LaunchConfiguration("action_timeout"),
                        "position_gain": LaunchConfiguration("position_gain"),
                        "rotation_gain": LaunchConfiguration("rotation_gain"),
                        "linear_scale": LaunchConfiguration("linear_scale"),
                        "max_linear_speed": LaunchConfiguration("max_linear_speed"),
                        "max_angular_speed": LaunchConfiguration("max_angular_speed"),
                    }
                ],
            ),
        ]
    )
