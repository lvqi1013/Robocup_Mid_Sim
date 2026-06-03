# 上层策略动作命令。
# nubot_control 发布该消息，nubot_hwcontroller_py 根据它计算底盘速度并输出带球/射门意图。
Point2D target         # 目标位置，通常为世界坐标系二维点
float32 target_ori     # 目标朝向，单位 rad
float32 target_w       # 目标角速度，旧项目中保留字段
Point2D target_vel     # 目标点自身速度或期望附加速度
float32 maxvel         # 最大平移速度，旧项目通常按 cm/s 语义使用
float32 maxw           # 最大角速度，单位 rad/s
Point2D robot_pos      # 当前机器人位置，世界坐标系
Point2D robot_vel      # 当前机器人速度，世界或本体语义由上层约定
float32 robot_ori      # 当前机器人朝向，单位 rad
float32 robot_w        # 当前机器人角速度，单位 rad/s
int8 move_action       # 平移动作编号，旧类型 char；枚举见 nubot_common/include/core.hpp
int8 rotate_action     # 旋转动作编号；旧字段名 rotate_acton，旧类型 char
int32 rotate_mode      # 旋转方向模式：0 最短方向，1 正方向，-1 负方向
int64 handle_enable    # 带球机构开关；0 关闭，非 0 开启
float32 strength       # 射门力度，请求射门时为非 0
int64 shoot_pos        # 射门机构位置/模式；旧字段名 shootPos
