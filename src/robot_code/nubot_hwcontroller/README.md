# nubot_hwcontroller

`nubot_hwcontroller_py` 是 `nubot_hwcontroller` 的 ROS 2/Python 迁移版。它保留旧节点的核心职责：接收上层 `ActionCmd`，计算机器人本体坐标系下的速度命令，并向 Gazebo Harmonic 运动适配层输出。

当前实现不再依赖 Gazebo Classic 的 `libnubot_gazebo.so`。旧插件里的职责被拆成 ROS 2 话题和 adapter 节点：

```text
运动：ActionCmd/VelCmd -> /nubot1/cmd_vel -> planar_motion_controller 或后续 ros2_control
带球：ActionCmd.handle_enable -> /nubot1/dribble_enable -> dribble_controller
射门：ActionCmd.strength -> /nubot1/shoot_power -> 后续 kick/shoot adapter
```

## 数据链路

```text
nubot_control_py 或测试节点
        -> /nubot1/nubotcontrol/actioncmd
        -> nubot_hwcontroller_py
        -> /nubot1/cmd_vel
        -> /nubot1/dribble_enable
        -> /nubot1/shoot_power
        -> planar_motion_controller
        -> Gazebo Harmonic /world/default/set_pose
```

如果 `nubot_interfaces` 已经迁移完成，节点会自动启用：

```text
订阅 /nubot1/nubotcontrol/actioncmd      nubot_interfaces/msg/ActionCmd
发布 /nubot1/nubotcontrol/velcmd        nubot_interfaces/msg/VelCmd
订阅 /nubot/redcard/chatter             nubot_interfaces/msg/SendingOff
```

如果 `nubot_interfaces` 暂时还不存在，节点仍可编译运行，并通过 `Twist` 覆盖输入测试：

```text
订阅 /nubot1/hwcontroller/cmd_vel_override    geometry_msgs/msg/Twist
发布 /nubot1/cmd_vel                          geometry_msgs/msg/Twist
```

始终启用的 ROS 2 执行话题：

```text
发布 /nubot1/cmd_vel                    geometry_msgs/msg/Twist
发布 /nubot1/dribble_enable             std_msgs/msg/Bool
发布 /nubot1/shoot_power                std_msgs/msg/Float32
订阅 /nubot1/ball_is_holding            std_msgs/msg/Bool
发布 /nubot1/hwcontroller/enabled       std_msgs/msg/Bool
```

## 编译

```bash
cd <ros2_ws>
colcon build --packages-select nubot_hwcontroller_py
source install/setup.bash
```

如果你已经有 `nubot_interfaces`，建议和它一起编译：

```bash
colcon build --packages-select nubot_interfaces nubot_hwcontroller_py
```

## 启动

```bash
ros2 launch nubot_hwcontroller_py hwcontroller.launch.py
```

常用参数：

```text
robot_name                  默认 nubot1
action_topic                默认 /nubot1/nubotcontrol/actioncmd
velcmd_topic                默认 /nubot1/nubotcontrol/velcmd
cmd_vel_topic               默认 /nubot1/cmd_vel
cmd_vel_override_topic      默认 /nubot1/hwcontroller/cmd_vel_override
dribble_enable_topic        默认 /nubot1/dribble_enable
shoot_power_topic           默认 /nubot1/shoot_power
ball_holding_topic          默认 /nubot1/ball_is_holding
redcard_topic               默认 /nubot/redcard/chatter
update_rate                 默认 100Hz
action_timeout              默认 0.25s
position_gain               位置 PD 的 P 项，默认 5.0
rotation_gain               朝向 P 项，默认 2.0
linear_scale                旧坐标速度到 m/s 的缩放，默认 0.01
max_linear_speed            输出 Twist 线速度限幅，默认 2.0m/s
max_angular_speed           输出 Twist 角速度限幅，默认 6.0rad/s
```

## 快速测试

先启动 Gazebo、`planar_motion_controller`，再启动本节点。没有 `ActionCmd` 时可以直接发布覆盖速度：

```bash
ros2 topic pub /nubot1/hwcontroller/cmd_vel_override geometry_msgs/msg/Twist \
  "{linear: {x: 0.4, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}" -r 20
```

观察输出：

```bash
ros2 topic echo /nubot1/cmd_vel
ros2 topic echo /nubot1/hwcontroller/enabled
```

如果同时运行了 `planar_motion_controller`，Gazebo 中的 `nubot1` 应该开始移动。

## 带球和射门

旧 Gazebo Classic 插件曾经直接订阅 `ActionCmd`：

```text
handle_enable -> dribble_req_
strength/shootPos -> shot_req_
```

迁移后这些执行意图由 `nubot_hwcontroller_py` 输出为普通 ROS 2 话题：

```text
/nubot1/dribble_enable   std_msgs/msg/Bool
/nubot1/shoot_power      std_msgs/msg/Float32
```

`/nubot1/dribble_enable` 已经可以接当前仓库里的 `nubot_gazebo_ros2/dribble_controller`。

`/nubot1/shoot_power` 是射门 adapter 的输入。后续可以实现一个 `kick_controller`：订阅 `/nubot1/shoot_power`、读取机器人和球的位姿，然后通过 Gazebo Harmonic 的力/速度接口或 Gazebo Sim System 插件给球施加速度。这样射门逻辑不再绑死在旧 Classic 插件里。

## 和旧 C++ 节点的对应关系

旧节点：

```text
/<robot>/nubotcontrol/actioncmd -> calculateSpeed() -> /<robot>/nubotcontrol/velcmd
```

新节点：

```text
/<robot>/nubotcontrol/actioncmd -> compute_twist_from_action() -> /<robot>/cmd_vel
                                                     -> /<robot>/nubotcontrol/velcmd
                                                     -> /<robot>/dribble_enable
                                                     -> /<robot>/shoot_power
```

其中 `VelCmd` 用于保持旧语义，`cmd_vel` 用于驱动当前 ROS 2/Gazebo 最小闭环。

## 当前边界

- 没有实现电机、电流、拨杆、真实全向轮反解。
- 射门目前输出 `/shoot_power`，具体给球加速度/力的部分应放在 Gazebo adapter 或 Gazebo Sim System 插件中。
- `ActionCmd` 中的位置量沿用旧项目语义，默认通过 `linear_scale=0.01` 转为 m/s。
- 红牌逻辑只保留 `id_maxvel_isvalid == robot_id` 禁用、`robot_id + 10` 恢复的简化行为。
- 最终物理控制应替换为 Gazebo Harmonic System plugin 或 `ros2_control`，但上层可以继续使用 `/nubotX/cmd_vel`、`/nubotX/dribble_enable`、`/nubotX/shoot_power` 或 `ActionCmd`。
