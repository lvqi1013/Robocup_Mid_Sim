import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('nubot_gazebo')
    model_share = get_package_share_directory('nubot_description')

    set_env = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f"{model_share}/models:$GZ_SIM_RESOURCE_PATH"
    )
    # 启动 Gazebo Harmonic (gz-sim)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ]),
        launch_arguments={
            'gz_args': os.path.join(pkg_share, 'worlds', 'robocup15MSL.sdf') #+ ' -v 4',
        }.items(),
    )
    
    bridge_config_path = os.path.join(pkg_share, 'config', 'bridge.yaml')
    ros_gz_bridge_node = Node(
        package = "ros_gz_bridge",
        executable = "parameter_bridge",
        name = "ros_gz_bridge",
        output = "screen",
        parameters = [
            {"config_file": bridge_config_path},
            ],
    )

    set_pose_service_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='set_pose',
        output = 'screen',
        arguments=[
                '/world/RoboCup15MSL/set_pose@ros_gz_interfaces/srv/SetEntityPose'
            ],
    )

    # ⚠️ 按需添加 ros_gz_bridge 桥接话题
    # bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    #     output='screen',
    # )

    return LaunchDescription([
        set_env,
        gz_sim,
        ros_gz_bridge_node,
        set_pose_service_node
    ])