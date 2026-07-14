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

## Gazebo 渲染器选择与闪退处理

仿真默认使用 **Ogre2**，正常情况下无需增加任何参数：

```bash
source install/setup.bash
ros2 launch nubot_gazebo load_world.launch.py
```

等价的显式写法为：

```bash
ros2 launch nubot_gazebo load_world.launch.py render_engine:=ogre2
```

在部分 ARM64 虚拟机、virgl/Mesa 虚拟显卡或 OpenGL 驱动环境中，Ogre2
可能无法编译 GLSL Shader。常见现象包括 Gazebo 窗口打开后立即关闭，以及终端中出现：

```text
Ogre::RenderingAPIException
Fragment Program 100000000PixelShader_ps failed to compile
GLSL compile log: 100000000PixelShader_ps
```

遇到上述无法显示或闪退问题时，将 `render_engine` 改为 `ogre`，使用
**Ogre 1.x 兼容渲染后端**：

```bash
source install/setup.bash
ros2 launch nubot_gazebo load_world.launch.py render_engine:=ogre
```

使用简化脚本时，也可以把相同参数传给 launch：

```bash
source gameReady.sh render_engine:=ogre
```

切回默认 Ogre2 时，去掉参数即可，也可以显式指定：

```bash
source gameReady.sh render_engine:=ogre2
```

启动日志可用于确认当前渲染器：

```text
Loading plugin [gz-rendering-ogre2]  # Ogre2，默认
Loading plugin [gz-rendering-ogre]   # Ogre 1.x，兼容模式
```

> `render_engine` launch 参数会传递为 Gazebo 的 `--render-engine` 命令行选项，
> 因此会覆盖 `~/.gz/sim/8/gui.config` 中保存的 `<engine>` 值。一般不需要手动
> 修改用户目录下的 Gazebo GUI 配置文件。

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
