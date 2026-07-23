import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('nubot_gazebo')
    model_share = get_package_share_directory('nubot_description')
    plugin_share = get_package_share_directory('nubot_plugin')
    plugin_lib_path = os.path.join(plugin_share, '..', '..', 'lib')

    # world_model 部分
    launch_world_model = LaunchConfiguration('launch_world_model') # 用来引用名为 'launch_world_model' 的启动参数（Launch Argument）的实时数值。
    team_prefix = LaunchConfiguration('team_prefix') # nubot or rival用于不同方启动 simulation interface alse
    team_size = LaunchConfiguration('team_size')# simulation interface alse

    # simulation interface part
    launch_simulation_interface = LaunchConfiguration('launch_simulation_interface')
    match_mode = LaunchConfiguration('match_mode')
    match_type = LaunchConfiguration('match_type')
    test_mode = LaunchConfiguration('test_mode')

    # # auto referee
    # launch_auto_referee = LaunchConfiguration('launch_auto_referee')

    # nubot_control部分
    # launch_nubot_control = LaunchConfiguration('launch_nubot_control')

    set_env = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(model_share, 'models'),
    )

    set_env2 = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=plugin_lib_path,
    )

    render_engine_arg = DeclareLaunchArgument(
        'render_engine',
        default_value='ogre',
        description=(
            'Gazebo compatibility rendering engine. The CPU launch defaults to '
            '"ogre" (Ogre 1.x).'
        ),
    )

    render_engine = LaunchConfiguration('render_engine')

    # 启动 Gazebo Harmonic (gz-sim)。默认使用 Ogre2；遇到虚拟机或显卡
    # 驱动兼容问题时，可通过 render_engine:=ogre 切换到 Ogre 1.x。
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ]),
        launch_arguments={
            'gz_args': [
                os.path.join(pkg_share, 'worlds', 'robocup15MSL.sdf'),
                ' -v 4 --render-engine ',
                render_engine,
            ],
        }.items(),
    )

    bridge_config_path = os.path.join(pkg_share, 'config', 'bridge.yaml')
    ros_gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[
            {'config_file': bridge_config_path},
        ],
    )

    set_pose_service_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='set_pose',
        output='screen',
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

    simulation_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('simulation_interface'),
                'launch', 'simulation_interface.launch.py'
            )
        ]),
        launch_arguments={
            'team_prefix': team_prefix,
            'team_size': team_size,
            'match_mode': match_mode,
            'match_type': match_type,
            'test_mode': test_mode,
        }.items(),
        condition=IfCondition(launch_simulation_interface),
    )

    # auto_referee_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(
    #             get_package_share_directory('auto_referee'),
    #             'launch', 'auto_referee.launch.py'
    #         )
    #     ]),
    #     launch_arguments={
    #         'cyan_prefix': team_prefix,
    #         'team_size': team_size,
    #     }.items(),
    #     condition=IfCondition(launch_auto_referee),
    # )

    # nubot_control_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(
    #             get_package_share_directory('nubot_control'),
    #             'launch', 'minimal_control.launch.py'
    #         )
    #     ]),
    #     launch_arguments={
    #         'team_prefix': team_prefix,
    #         'team_size': team_size,
    #     }.items(),
    #     condition=IfCondition(launch_nubot_control),
    # )

    return LaunchDescription([
        render_engine_arg,

        # world model part
        DeclareLaunchArgument('launch_world_model', default_value='true'),
        DeclareLaunchArgument('team_prefix', default_value='nubot'),
        DeclareLaunchArgument('team_size', default_value='5'),

        # simulation interface
        DeclareLaunchArgument('launch_simulation_interface', default_value='true'),
        DeclareLaunchArgument('match_mode', default_value='0'),
        DeclareLaunchArgument('match_type', default_value='0'),
        DeclareLaunchArgument('test_mode', default_value='0'),     

        DeclareLaunchArgument('launch_auto_referee', default_value='true'),

        # nubot control
        # DeclareLaunchArgument('launch_nubot_control', default_value='true'),

        # launch gazebo part
        set_env,
        set_env2,
        gz_sim,
        ros_gz_bridge_node,
        set_pose_service_node,

        simulation_interface_launch,
        # auto_referee_launch,
        # world model part
        world_model_launch,
        # nubot_control_launch,
    ])
