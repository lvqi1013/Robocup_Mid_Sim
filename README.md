![比赛创建界面](./pics/first_ui.png)

# 编译工作空间

```bash
colcon build
```

# 加载比赛场地

## 常规(一条一条命令加载)

```bash
source install/setup.bash
ros2 launch nubot_gazebo load_world.launch.py
```

## 简化(运行定义好的脚本文件)

```bash
source gameReady.sh
```

```bash
src/
    nubot_interfaces/ # msg/srv/action
    nubot_common/ # core.hpp、几何工具、公共 C++ 库
    nubot_description/ # SDF/mesh/world/resource
    nubot_gazebo_sim/ # Gazebo Harmonic System 插件
    world_model/ # ROS2 rclcpp 节点
    nubot_control/ # ROS2 rclcpp 节点
    nubot_hwcontroller/ # ROS2 硬控/仿真控制适配
    auto_referee/ # ROS2 裁判节点或 Gazebo System
    nubot_coach/ # Qt + ROS2，后迁移
    nubot_bringup/ # launch.py、参数、组合启动
```

# 软件模块 Software Modeules

该软件包括了gazebo_visual模块

## gazebo_visual模块

包含足球模型，机器人模型，Gazebo仿真平台，这部分选手不应做任何改动，最终版本以比赛时的版本为准。
