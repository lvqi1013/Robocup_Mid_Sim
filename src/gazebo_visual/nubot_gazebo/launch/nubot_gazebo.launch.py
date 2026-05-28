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


def _spawn_entities(context, *args, **kwargs):
    """
    动态生成所有 spawn 动作
    完全替代原 robot_up.sh 中的循环 + sleep 逻辑
    """
    cfg = _load_config()
    actions = []
    model_share = get_package_share_directory('nubot_description')

    # === 1. Spawn Football ===
    football_sdf = os.path.join(model_share, 'models', 'football', 'model.sdf')
    actions.append(Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', football_sdf,
            '-name', cfg['football']['name'],
            '-x', '0.0', '-y', '0.0', '-z', '0.0',
        ],
        output='screen',
    ))

    # === 2. Spawn Cyan Robots ===
    cyan_cfg = cfg['cyan']
    for i in range(cyan_cfg['num']):
        pose = cyan_cfg['initial_poses'][i]
        model_sdf = os.path.join(model_share, 'models', f'nubot{i+1}', 'model.sdf')
        actions.append(Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-file', model_sdf,
                '-name', f"{cyan_cfg['prefix']}{i + 1}",
                '-x', str(pose['x']),
                '-y', str(pose['y']),
                '-z', '0.0',
                '-R', '0.0', '-P', '0.0', '-Y', str(pose['yaw']),
            ],
            output='screen',
        ))

    # === 3. Spawn Magenta Robots ===
    mag_cfg = cfg['magenta']
    for i in range(mag_cfg['num']):
        pose = mag_cfg['initial_poses'][i]
        model_sdf = os.path.join(model_share, 'models', f'rival{i+1}', 'model.sdf')
        actions.append(Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-file', model_sdf,
                '-name', f"{mag_cfg['prefix']}{i + 1}",
                '-x', str(pose['x']),
                '-y', str(pose['y']),
                '-z', '0.0',
                '-R', '0.0', '-P', '0.0', '-Y', str(pose['yaw']),
            ],
            output='screen',
        ))

    return actions


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

    # 动态生成 spawn 节点（替代 robot_up.sh）
    spawn_entities = OpaqueFunction(function=_spawn_entities)

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
        spawn_entities,
        # bridge,
    ])