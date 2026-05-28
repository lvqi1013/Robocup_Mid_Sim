import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _load_config():
    """加载全局配置文件"""
    pkg_share = get_package_share_directory('nubot_gazebo')
    config_path = os.path.join(pkg_share, 'config', 'global_config.yaml')
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)




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
            'gz_args': os.path.join(pkg_share, 'worlds', 'robocup15MSL.sdf'),
        }.items(),
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
        # bridge,
    ])