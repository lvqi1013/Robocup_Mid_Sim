import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    model_share = get_package_share_directory('nubot_description')
    pkg_share = get_package_share_directory('nubot_gazebo')

    print(model_share)

    print(pkg_share)
    return LaunchDescription([
    ])
