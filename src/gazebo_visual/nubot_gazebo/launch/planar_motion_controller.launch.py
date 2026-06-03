from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("world_name", default_value="RoboCup15MSL"),
            DeclareLaunchArgument("robot_name", default_value="nubot1"),
            DeclareLaunchArgument("pose_topic", default_value="/world/default/dynamic_pose/info"),
            DeclareLaunchArgument("robot_pose_topic", default_value="/nubot1/pose"),
            DeclareLaunchArgument("use_model_pose_topics", default_value="true"),
            DeclareLaunchArgument("model_pose_topic_type", default_value="pose_array"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/nubot1/cmd_vel"),
            DeclareLaunchArgument("pose_feedback_topic", default_value="/nubot1/pose_feedback"),
            DeclareLaunchArgument("motion_active_topic", default_value="/nubot1/motion_active"),
            DeclareLaunchArgument("update_rate", default_value="50.0"),
            DeclareLaunchArgument("command_timeout", default_value="0.25"),
            DeclareLaunchArgument("max_linear_speed", default_value="2.0"),
            DeclareLaunchArgument("max_angular_speed", default_value="6.0"),
            Node(
                package="nubot_gazebo",
                executable="planar_motion_controller",
                name=[LaunchConfiguration("robot_name"), "_planar_motion_controller"],
                output="screen",
                parameters=[
                    {
                        "world_name": LaunchConfiguration("world_name"),
                        "robot_name": LaunchConfiguration("robot_name"),
                        "pose_topic": LaunchConfiguration("pose_topic"),
                        "robot_pose_topic": LaunchConfiguration("robot_pose_topic"),
                        "use_model_pose_topics": LaunchConfiguration("use_model_pose_topics"),
                        "model_pose_topic_type": LaunchConfiguration("model_pose_topic_type"),
                        "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                        "pose_feedback_topic": LaunchConfiguration("pose_feedback_topic"),
                        "motion_active_topic": LaunchConfiguration("motion_active_topic"),
                        "update_rate": LaunchConfiguration("update_rate"),
                        "command_timeout": LaunchConfiguration("command_timeout"),
                        "max_linear_speed": LaunchConfiguration("max_linear_speed"),
                        "max_angular_speed": LaunchConfiguration("max_angular_speed"),
                    }
                ],
            ),
        ]
    )
