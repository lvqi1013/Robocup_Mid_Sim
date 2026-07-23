# nubot_control

`nubot_control` 是 RoboCup 中型组仿真比赛中的上层比赛控制节点。它读取
`world_model` 输出的世界模型，根据当前裁判指令和机器人状态生成 `ActionCmd`，
再交给 `nubot_hwcontroller` 转换成 Gazebo Harmonic 使用的 `VelCmd`。

本版本迁移自 `simatch/src/robot_code/nubot_control`。迁移原则是只实现 ROS1
代码当前真正运行的简单逻辑，不补写原工程留给参赛者的复杂角色策略。

## 当前行为

- `STOPROBOT`：停止平移和旋转。
- 定位球阶段：使用 ROS1 中的五机器人固定站位。
- `PARKINGROBOT`：沿场地下边界排队停车。
- 正常比赛：除守门员外，距离球最近的有效机器人作为主攻。
- 主攻未持球：先朝向足球，再接近足球并请求带球。
- 主攻持球：移动到 `(200, 300)` cm，面向对方球门后射门。
- 世界模型超过 0.25 秒未更新：发布停止动作。

`ActiveRole`、`AssistRole`、`PassiveRole`、`MidfieldRole`、`GoalieStrategy`
及 `ParabolaFitter3D` 均已按 ROS1 原名保留。守门员、助攻、中场、防守、动态
传球、避障和路径规划在 ROS1 源码中没有完整实现或没有接入主循环，因此对应
类保留原成员与接口，未完成角色的 `process()` 输出安全停止，但不虚构比赛策略。
详细映射见
[`docs/CODE_ROLES.md`](docs/CODE_ROLES.md)。

## 启动

完整仿真启动文件已经同时接入 `nubot_control` 和 `nubot_hwcontroller`：

```bash
source install/setup.bash
ros2 launch nubot_gazebo load_world_gpu.launch.py
# 或使用 CPU/Ogre 渲染：
ros2 launch nubot_gazebo load_world_cpu.launch.py
```

只启动一整队的上层策略：

```bash
source install/setup.bash
ros2 launch nubot_control nubot_control.launch.py
```

单机器人启动：

```bash
ros2 run nubot_control nubot_control_node \
  --ros-args -p robot_name:=nubot2 -p team_prefix:=nubot
```

## 控制链

```text
Gazebo Harmonic nubot_plugin
  -> world_model
  -> nubot_control
  -> nubot_hwcontroller
  -> Gazebo Harmonic nubot_plugin
```
