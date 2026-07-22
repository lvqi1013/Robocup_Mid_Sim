import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    pkg_share = get_package_share_directory('nubot_gazebo')
    model_share = get_package_share_directory('nubot_description')
    plugin_share = get_package_share_directory('nubot_plugin')
    plugin_lib_path = os.path.join(plugin_share, '..', '..', 'lib')

    # world_model 部分
    launch_world_model = LaunchConfiguration('launch_world_model') # 用来引用名为 'launch_world_model' 的启动参数（Launch Argument）的实时数值。
    team_prefix = LaunchConfiguration('team_prefix') # nubot or rival用于不同方启动
    team_size = LaunchConfiguration('team_size')


    set_env = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(model_share, "models"),
    )

    set_env2 = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=plugin_lib_path,
    )

    # GPU 模式：显式使用 Gazebo 默认的 Ogre2 渲染后端。
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ]),
        launch_arguments={
            'gz_args': (
                os.path.join(pkg_share, 'worlds', 'robocup15MSL.sdf')
                + ' -v 4'
            ),
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

    world_model_launch = IncludeLaunchDescription(# 嵌套另一个launch文件
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('world_model'),
                'launch', 'sim_world_model.launch.py'
            )
        ]),
        launch_arguments={
            'team_prefix': team_prefix,
            'team_size': team_size,
        }.items(), # 作用：向被调用的子 Launch 文件传递启动参数。
        condition=IfCondition(launch_world_model),
    )

    return LaunchDescription([
        # world model part
        DeclareLaunchArgument('launch_world_model', default_value='true'),
        DeclareLaunchArgument('team_prefix', default_value='nubot'),
        DeclareLaunchArgument('team_size', default_value='5'),

        # launch gazebo part
        set_env,
        set_env2,
        gz_sim,
        ros_gz_bridge_node,
        set_pose_service_node,

        world_model_launch,

    ])