# Nubot Gazebo 插件说明

本文档根据 [`nubot_plugin.hh`](./nubot_plugin.hh) 和
[`nubot_plugin.cc`](./nubot_plugin.cc) 的当前实现整理。

## 1. 插件概述

`nubot_plugins::NubotGazebo` 是挂载在单个机器人模型上的 Gazebo Sim System 插件，
同时实现 `ISystemConfigure`、`ISystemPreUpdate` 和 `ISystemPostUpdate`。每个机器人模型
各自创建一个 ROS 2 节点，节点名为 `nubot_gazebo_<model_name>`。

插件位于 Gazebo 仿真与 Nubot ROS 2 控制程序之间，主要实现：

- 接收机器人平移、旋转速度命令，并直接更新模型的线速度和角速度组件；
- 使用四轮全向底盘运动学限制轮速、加速度和减速度；
- 接收带球和射门命令，模拟地面射门、挑射及带球；
- 读取 Gazebo 中机器人、足球和其他机器人的状态，生成模拟全向视觉信息；
- 判断足球是否位于机器人的可控球区域，并发布持球状态；
- 接收比赛模式和红牌/恢复上场消息；
- 为两支队伍提供统一的坐标视角，并可向观测数据加入随机噪声；
- 判断机器人是否越出有效场地区域，以及机器人是否可能被卡住。

插件注册名为 `nubot_plugins::NubotGazebo`，动态库通常以如下方式加载：

```xml
<plugin filename="libnubot_plugin.so" name="nubot_plugins::NubotGazebo">
  <ball_name>football</ball_name>
  <cyan_prefix>nubot</cyan_prefix>
  <magenta_prefix>rival</magenta_prefix>
  <flip_cord>false</flip_cord>
</plugin>
```

## 2. Gazebo 更新流程

### [`Configure`](./nubot_plugin.cc#L73)

插件加载时执行以下初始化：

1. 保存当前机器人模型实体并读取模型名；
2. 读取 SDF 参数，根据模型名前缀解析机器人编号 `agent_id_`；
3. 确保机器人具有 `LinearVelocity` 和 `AngularVelocity` 组件；
4. 在场景中查找足球实体；
5. 创建 ROS 2 节点、发布者、订阅者、服务客户端和独立执行线程。

### [`PreUpdate`](./nubot_plugin.cc#L113)

在每个未暂停的仿真步、物理更新之前执行控制：

1. 继续查找尚未出现的足球实体；
2. 执行红牌消息产生的待处理机器人位置重置，并清零速度；
3. 收到过速度命令后，通过 `nubot_locomotion` 更新机器人速度；
4. 通过 `nubot_be_control` 处理带球与射门。

### [`PostUpdate`](./nubot_plugin.cc#L144)

在物理更新之后读取各模型的位姿和速度，整理足球、障碍物及本队机器人信息，然后发布
模拟视觉和持球消息。ROS 2 回调线程与 Gazebo 更新线程通过 `mutex_` 保护共享状态。

## 3. ROS 2 接口

除 `/DribbleId` 外，机器人专属话题使用模型名作为命名空间。例如模型名为 `nubot1`
时，`/<model_name>/nubotcontrol/velcmd` 展开为 `/nubot1/nubotcontrol/velcmd`。
所有发布者的 QoS depth 为 `10`，所有订阅者的 QoS depth 为 `100`。

### 3.1 发布的话题

| 话题 | 消息类型 | 作用 |
| --- | --- | --- |
| `/<model_name>/omnivision/OmniVisionInfo` | `nubot_interfaces/msg/OminiVisionInfo` | 发布模拟全向视觉数据，包括球、本队机器人和其他机器人形成的障碍物信息。 |
| `/<model_name>/ballisholding/BallIsHolding` | `nubot_interfaces/msg/BallIsHolding` | 发布足球是否同时满足控球距离和控球角度条件。它表示“球在可控区域”，不等同于已经收到带球命令。 |

接口创建位置见 [`init_ros`](./nubot_plugin.cc#L155)，消息组装位置见
[`message_publish`](./nubot_plugin.cc#L584)。

#### `OminiVisionInfo` 的内容

- `ballinfo`：足球世界坐标、相对机器人的极坐标、世界坐标速度以及信息有效标志；
- `robotinfo`：当前队伍所有机器人编号、位置、朝向、速度、有效性，以及本机的卡住和带球状态；
- `obstacleinfo`：除本机外所有机器人的世界坐标和相对本机的极坐标。

位置和线速度大多从 Gazebo 的米制单位乘以 `100` 后发布，即按厘米和厘米每秒表达；
角度和角速度使用弧度。当前实现中 `obstacleinfo.polar_pos.radius` 直接来自 Gazebo 距离，
没有乘以 `100`，因此其半径仍为米。

### 3.2 订阅的话题

| 话题 | 消息类型 | 回调 | 作用 |
| --- | --- | --- | --- |
| `/<model_name>/nubotcontrol/velcmd` | `nubot_interfaces/msg/VelCmd` | [`vel_cmd_cb`](./nubot_plugin.cc#L252) | 接收本体坐标系 `vx`、`vy` 和绕 z 轴角速度 `w`。平移速度由 cm/s 转为 m/s，再根据机器人朝向转换为世界坐标速度；翻转队伍会反转 `vx`、`vy`。 |
| `/<model_name>/nubotcontrol/actioncmd` | `nubot_interfaces/msg/ActionCmd` | [`action_cmd_cb`](./nubot_plugin.cc#L278) | 使用 `handle_enable` 控制带球请求，使用 `strength` 表示射门力度，使用 `shoot_pos` 选择射门模式。只有球已在可控区域时，非零力度才会生成射门请求。 |
| `/<cyan_prefix>/receive_from_coach` | `nubot_interfaces/msg/CoachInfo` | [`coach_info_cb`](./nubot_plugin.cc#L297) | 读取 `match_mode`。当模式为 `STOPROBOT` 时禁止带球和射门。默认前缀下为 `/nubot/receive_from_coach`；当前实现中两队实例都订阅这一话题。 |
| `/<cyan_prefix>/redcard/chatter` | `nubot_interfaces/msg/SendingOff` | [`sending_off_cb`](./nubot_plugin.cc#L303) | 接收蓝/青队罚下或恢复消息；消息队伍和球员编号匹配本机时，将机器人移至代码指定的场外位置。 |
| `/<magenta_prefix>/redcard/chatter` | `nubot_interfaces/msg/SendingOff` | [`sending_off_cb`](./nubot_plugin.cc#L303) | 接收红/品红队罚下或恢复消息，处理逻辑同上。 |

红牌回调不直接在 ROS 2 线程中修改 Gazebo ECS，而是写入待处理位姿；下一次
`PreUpdate` 再移动模型并清零线速度和角速度，避免跨线程操作实体组件。

### 3.3 使用的服务

| 名称 | 类型 | 角色 | 作用 |
| --- | --- | --- | --- |
| `/DribbleId` | `nubot_interfaces/srv/DribbleId` | 客户端 | 机器人刚进入带球状态时上报持球者编号；翻转队伍上报 `agent_id + 5`。退出带球时发送 `-1` 清空持球者。请求为异步调用且服务必须已就绪。 |

该插件只创建 `/DribbleId` 客户端，不创建任何 ROS 2 服务端。若服务不可用，插件仍可在
Gazebo 中带球，但不会成功同步全局持球者编号。

## 4. 主要功能实现

### 4.1 机器人运动控制

[`nubot_locomotion`](./nubot_plugin.cc#L485) 将目标平移和旋转速度转换为四个全向轮的
等效轮速，并依次执行：

1. [`speed_limit`](./nubot_plugin.cc#L708)：按比例缩放四轮速度，使轮速绝对值不超过
   `5.0`；
2. [`accelerate_limit`](./nubot_plugin.cc#L741)：限制加速度为 `2.5`、减速度为 `5.0`；
3. 再次执行轮速限制；
4. 将结果写入机器人实体的线速度和角速度组件。

模型只保留平面内的 x、y 平移和绕 z 轴旋转。`last_command_time_sec_` 用于判断是否收到过
速度命令；当前代码没有命令超时清零机制，所以最后一次目标速度会持续生效。

### 4.2 控球条件判断

[`get_is_hold_ball`](./nubot_plugin.cc#L624) 根据机器人到球的平面距离和球相对机器人正前方
的夹角判断是否能够持球：

- 距离不超过 `dribble_distance_thres`，默认 `0.50 m`；
- 夹角位于 `±dribble_angle_thres / 2` 内，默认总角度为 `30°`，即 `±15°`。

### 4.3 带球与行为控制

[`nubot_be_control`](./nubot_plugin.cc#L455) 在机器人高度小于 `0.05 m` 时处理动作。
带球必须同时满足：已请求带球、球在可控区域、比赛模式不是 `STOPROBOT`。

[`dribble_ball`](./nubot_plugin.cc#L539) 每个控制周期将球放到机器人朝向前方 `0.35 m`、
高度 `0.12 m` 处，并清零球实体的线速度，从而模拟足球被带球机构保持。进入和退出带球
状态时会分别通过 `/DribbleId` 上报机器人编号和 `-1`。

`nubot_be_control` 会根据机器人高度更新 `can_move_`，但当前版本的
`nubot_locomotion` 没有读取该变量。因此它目前不会实际阻止速度组件写入，只会影响内部状态。

### 4.4 射门

[`kick_ball`](./nubot_plugin.cc#L558) 直接设置足球的线速度：

- `mode == 1`（`RUN`）：地面射门，沿机器人前方向发球，力度上限为 `10.0`；
- `mode == -1`（`FLY`）：挑射，水平分量为 `0.8 × strength`，竖直分量为
  `0.6 × strength`；
- 其他模式：记录错误日志，不改变足球速度。

射门要求球在可控区域且比赛模式不是 `STOPROBOT`。请求处理后 `shot_req_` 被清除，
因此每条有效动作命令只触发一次射门。

### 4.5 模拟感知

[`update_model_info`](./nubot_plugin.cc#L344) 遍历 Gazebo 中名称以 `cyan_prefix` 或
`magenta_prefix` 开头的模型以及足球模型，读取其世界位姿、线速度和角速度。处理结果包括：

- 计算本机到球的向量、距离和机器人正前方向；
- 将除本机外的所有机器人作为障碍物；
- 只把与本机同队的机器人写入 `robotinfo`；
- 检查机器人是否在场地边界额外放宽 `1 m` 的有效范围内；
- 比较期望速度和实际速度，连续多次偏差过大时将本机标记为卡住。

每次读取模型的 x、y 位置和线速度时，都可能按 `noise_rate` 概率加入标准差由
`noise_scale` 缩放的高斯噪声。

### 4.6 坐标翻转

当 `flip_cord=true` 时，插件对读取到的 x、y 位置和线速度取反，使对方队伍也能从己方
进攻方向观察场地；速度命令、带球位置和射门方向会执行相应反向变换。默认配置中
`nubot*` 使用 `false`，`rival*` 使用 `true`。

## 5. SDF 参数

| 参数 | 默认值 | 作用 |
| --- | ---: | --- |
| `ball_name` | `football` | 足球模型名称。 |
| `cyan_prefix` | `nubot` | 蓝/青队机器人模型名前缀和部分队伍级话题前缀。 |
| `magenta_prefix` | `rival` | 红/品红队机器人模型名前缀和红牌话题前缀。 |
| `dribble_distance_thres` | `0.50` | 可控球最大距离，单位 m。 |
| `dribble_angle_thres` | `30.0` | 可控球扇区总角度，单位度。 |
| `field_length` | `22.0` | 场地长度，单位 m，用于机器人有效性判断。 |
| `field_width` | `14.0` | 场地宽度，单位 m，用于机器人有效性判断。 |
| `noise_scale` | `0.10` | 高斯噪声缩放系数。 |
| `noise_rate` | `0.01` | 每个受影响分量在一次更新中加入噪声的概率。 |
| `flip_cord` | `false` | 是否启用两队坐标翻转。 |

机器人编号取自模型名前缀后的第一个字符，例如 `nubot3` 得到编号 `3`。因此当前解析方式
只适合一位数编号。

## 6. 代码结构索引

- [`nubot_plugin.hh`](./nubot_plugin.hh)：插件类、ROS 2 接口、Gazebo 实体、状态和参数声明；
- [`Configure`](./nubot_plugin.cc#L73)：SDF、实体和 ROS 2 初始化；
- [`init_ros`](./nubot_plugin.cc#L155)：创建所有 ROS 2 接口及执行线程；
- [`PreUpdate`](./nubot_plugin.cc#L113)：应用运动、位置重置、带球和射门控制；
- [`PostUpdate`](./nubot_plugin.cc#L144)：采集仿真状态并发布消息；
- [`update_model_info`](./nubot_plugin.cc#L344)：构造足球、机器人和障碍物信息；
- [`nubot_locomotion`](./nubot_plugin.cc#L485)：底盘运动和速度/加速度限制；
- [`nubot_be_control`](./nubot_plugin.cc#L455)：带球、射门及比赛状态控制；
- [`dribble_ball`](./nubot_plugin.cc#L539)：模拟带球；
- [`kick_ball`](./nubot_plugin.cc#L558)：模拟地面射门和挑射；
- [`message_publish`](./nubot_plugin.cc#L584)：发布全向视觉及持球状态。
