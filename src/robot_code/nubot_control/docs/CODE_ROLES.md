# nubot_control 文件角色与 ROS1 迁移映射

本文档说明迁移后每个代码文件在 RoboCup 中型组仿真比赛系统中的角色，同时
记录 `simatch` ROS1 源码的去向。项目位置和速度继续使用旧系统约定的 cm、cm/s，
角度和角速度使用 rad、rad/s。

## 运行链路

```text
/<robot>/worldmodel/worldmodelinfo
                  |
                  v
          control_node.py
                  |
                  v
            strategy.py
                  |
                  v
/<robot>/nubotcontrol/actioncmd
                  |
                  v
        nubot_hwcontroller
                  |
                  v
/<robot>/nubotcontrol/velcmd -> Gazebo Harmonic
```

## Python 源文件

| 文件 | 仿真系统中的角色 | 主要输入/输出 | ROS1 来源 |
|---|---|---|---|
| `nubot_control/__init__.py` | 标识 Python 包，不包含运行逻辑。 | 无 | ROS2 包结构新增。 |
| `nubot_control/constants.py` | 统一保存比赛模式、动作、角色编号和速度上限，避免旧工程多份 `define.hpp` 相互冲突。 | 被其他 Python 模块导入。 | `nubot_common/core.hpp`、`define.hpp`、`strategydefine.hpp`。 |
| `nubot_control/geometry.py` | 提供策略需要的二维点、距离、方向和角度归一化。 | Python 数据对象。 | `DPoint.hpp` 中实际使用的子集。 |
| `nubot_control/field.py` | 保存球门关键点和己方禁区判断。只迁移当前站位、射门会使用的场地信息。 | Python 数据对象。 | `fieldinformation.h/.cpp` 的运行子集。 |
| `nubot_control/models.py` | 定义与 ROS 消息解耦的机器人、足球、世界状态和动作决策。 | `WorldState`、`ActionDecision`。 | 原 `World_Model_Info` 与 `ActionCmd` 临时状态。 |
| `nubot_control/activerole.py` | `ActiveRole` 主攻角色，执行追球、带球到固定点、转向和射门。 | `WorldState` -> `ActionDecision`。 | `activerole.cpp/.h` 的状态初始化，以及 `nubot_control.cpp` 中真正生效的主动行为。 |
| `nubot_control/assistrole.py` | `AssistRole` 助攻角色，保留原成员和同名接口；未完成策略当前输出安全停止。 | `WorldState` -> 停止决策。 | `assistrole.cpp/.h`。 |
| `nubot_control/passiverole.py` | `PassiveRole` 防守角色，保留原成员和同名接口；未完成策略当前输出安全停止。 | `WorldState` -> 停止决策。 | `passiverole.cpp/.h`。 |
| `nubot_control/midfieldrole.py` | `MidfieldRole` 中场角色，保留助攻/防守模式接口；未完成策略当前输出安全停止。 | `WorldState` -> 停止决策。 | `midfieldrole.cpp/.h`。 |
| `nubot_control/goaliestrategy.py` | 保留 `GoalieStrategy`、`GoalieState` 和 `ParabolaFitter3D` 原名及初始化状态；未完成的守门和三维球预测当前使用安全结果。 | `WorldState` -> 守门员停止决策。 | `goaliestrategy.cpp/.h`。 |
| `nubot_control/strategy.py` | 比赛模式总调度：定位、停车、最近机器人选择，并把主动和守门行为委托给对应角色类。 | `WorldState` -> `ActionDecision`。 | `nubot_control.cpp` 中 `positioning`、`parking`、`normalGame`、`move2target`、`move2ori`。 |
| `nubot_control/control_node.py` | ROS2 单机器人节点。订阅世界模型和持球状态，调用策略并发布动作与协同状态；世界模型超时时安全停车。 | 订阅 `WorldModelInfo`、`BallIsHolding`；发布 `ActionCmd`、`StrategyInfo`。 | `nubot_control.cpp` 的 ROS 通信、定时器、消息转换部分。 |

## Launch、包和测试文件

| 文件 | 角色 |
|---|---|
| `launch/nubot_control.launch.py` | 按 `team_prefix` 和 `team_size` 为整支队伍创建独立控制节点。 |
| `setup.py` | 安装 Python 包、Launch、文档并注册 `nubot_control_node`。 |
| `setup.cfg` | 把 ROS2 可执行脚本安装到 `lib/nubot_control`。 |
| `package.xml` | 声明 ament_python、rclpy、launch 和 nubot_interfaces 依赖。 |
| `resource/nubot_control` | ament 索引标记，使 ROS2 能发现该包。 |
| `README.md` | 面向使用者说明当前能力、限制、启动方式和控制链。 |
| `docs/CODE_ROLES.md` | 本文件，供参赛者理解每个文件和旧源码的对应关系。 |
| `test/test_geometry.py` | 验证点在线上移动和跨 ±pi 角度归一化。 |
| `test/test_strategy.py` | 验证停止、定位、主攻选择、追球和持球动作。 |
| `test/test_roles.py` | 验证五个 ROS1 同名角色可导入、可实例化，且未完成角色输出安全停止。 |
| `test/test_copyright.py` | ROS2 标准版权检查入口。 |
| `test/test_flake8.py` | ROS2 标准 Python 代码规范检查入口。 |
| `test/test_pep257.py` | ROS2 标准文档字符串检查入口。 |

## ROS1 文件处理结果

以下文件没有按“一份 C++ 对应一份 Python”机械复制，因为原排布会继续制造耦合。

| ROS1 文件 | 处理结果与原因 |
|---|---|
| `src/nubot_control.cpp` | 拆成 `control_node.py` 和 `strategy.py`。前者只负责 ROS，后者只负责策略。 |
| `src/fieldinformation.cpp`、`fieldinformation.h` | 当前使用部分进入 `field.py`；未使用的大量线段/区域数据不重复迁移。 |
| `common.hpp` | 当前策略只使用的点、距离和角度功能进入 `geometry.py`；其余约 600 行未调用函数不迁移。 |
| `world_model_info.h` | 消息缓存部分进入 `models.py` 和 `control_node.py`；未被当前主循环调用的传球视野搜索不迁移。 |
| `strategy.cpp/.hpp` | 只有构造和指针装配，且主循环未调用 `process()`，不建立无效 Python 外壳。 |
| `role_assignment.cpp/.h` | 只有构造函数，效能矩阵方法没有实现；当前最近球机器人选择保留在 `strategy.py`。 |
| `activerole.cpp/.h` | 迁为同名 `ActiveRole`；保留构造状态，并接收原主循环中实际生效的主动行为。头文件中未实现的复杂策略仍不虚构。 |
| `assistrole.cpp/.h` | 迁为同名 `AssistRole`；保留成员和公开接口，`process()` 当前明确输出安全停止。 |
| `midfieldrole.cpp/.h` | 迁为同名 `MidfieldRole`；保留成员和公开接口，`process()` 当前明确输出安全停止。 |
| `passiverole.cpp/.h` | 迁为同名 `PassiveRole`；保留成员和公开接口，`process()` 当前明确输出安全停止。 |
| `passstrategy.cpp/.h` | `process()` 没有实现，不迁移空类。 |
| `plan.cpp/.h/.hpp`、`behaviour.cpp/.hpp` | 路径规划和行为接口未形成当前可运行闭环，不迁移未接入代码。 |
| `subtargets.cpp/.h`、`bezier.cpp/.h` | 实现文件为空或只有构造函数，当前主循环没有使用。 |
| `goaliestrategy.cpp/.h` | 迁为同名 `GoalieStrategy` 和 `ParabolaFitter3D`；保留初始状态和接口，当前 1 号机器人安全停止。 |
| `staticpass.cpp/.h` | 有部分静态传球计算，但 `nubot_control.cpp` 当前固定站位没有调用它，因此不接入运行链。 |
| `DebugInfo.msg` | 当前节点没有发布该调试消息；ROS2 已使用日志和标准策略消息，不新增未使用接口。 |

## 参赛开发建议

后续补写比赛策略时，直接扩展对应的 `activerole.py`、`assistrole.py`、
`passiverole.py`、`midfieldrole.py` 或 `goaliestrategy.py`，并保持 `strategy.py`
只负责角色调度、`control_node.py` 只负责消息转换。这样可以在没有 Gazebo 的
情况下为各角色编写单元测试，也能避免再次形成 ROS1 版本的循环依赖和大文件结构。
