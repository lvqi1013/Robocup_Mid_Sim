source install/setup.bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/lq/code/Robocup_Mid_Sim/install/gz_hello_plugin/lib:$GZ_SIM_SYSTEM_PLUGIN_PATH
ros2 launch nubot_gazebo load_world.launch.py 