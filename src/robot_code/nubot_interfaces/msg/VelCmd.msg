# 底盘速度命令。
# 由 nubot_hwcontroller_py 根据 ActionCmd 解算得到，也可作为兼容旧模块的速度接口。
float32 vx  # 机器人本体坐标系 x 方向速度；旧字段名 Vx
float32 vy  # 机器人本体坐标系 y 方向速度；旧字段名 Vy
float32 w   # 绕 z 轴角速度，单位 rad/s
