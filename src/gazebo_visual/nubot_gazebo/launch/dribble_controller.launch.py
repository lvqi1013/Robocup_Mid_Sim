from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("world_name", default_value="RoboCup15MSL"),
            DeclareLaunchArgument("robot_name", default_value="nubot1"),
            DeclareLaunchArgument("ball_name", default_value="football"),
            DeclareLaunchArgument("pose_topic", default_value="/world/default/dynamic_pose/info"),
            DeclareLaunchArgument("robot_pose_topic", default_value="/nubot1/pose"),
            DeclareLaunchArgument("ball_pose_topic", default_value="/football/pose"),
            DeclareLaunchArgument("use_model_pose_topics", default_value="true"),
            DeclareLaunchArgument("model_pose_topic_type", default_value="pose_array"),
            DeclareLaunchArgument("dribble_enable_topic", default_value="/nubot1/dribble_enable"),
            DeclareLaunchArgument("holding_topic", default_value="/nubot1/ball_is_holding"),
            DeclareLaunchArgument("debug_topic", default_value="/nubot1/dribble_debug"),
            Node(
                package="nubot_gazebo",
                executable="dribble_controller",
                name="nubot1_dribble_controller",
                output="screen",
                parameters=[
                    {
                        "world_name": LaunchConfiguration("world_name"),
                        "robot_name": LaunchConfiguration("robot_name"),
                        "ball_name": LaunchConfiguration("ball_name"),
                        "pose_topic": LaunchConfiguration("pose_topic"),
                        "robot_pose_topic": LaunchConfiguration("robot_pose_topic"),
                        "ball_pose_topic": LaunchConfiguration("ball_pose_topic"),
                        "use_model_pose_topics": LaunchConfiguration("use_model_pose_topics"),
                        "model_pose_topic_type": LaunchConfiguration("model_pose_topic_type"),
                        "dribble_enable_topic": LaunchConfiguration("dribble_enable_topic"),
                        "holding_topic": LaunchConfiguration("holding_topic"),
                        "debug_topic": LaunchConfiguration("debug_topic"),
                    }
                ],
            ),
        ]
    )
