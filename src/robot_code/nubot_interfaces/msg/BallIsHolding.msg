# 机器人是否处于持球/带球状态。
# ROS2 中推荐使用 bool；这里保持 uint8 以兼容旧 BallIsHolding 消息的数值语义。
uint8 ball_is_holding  # 0 表示未持球，非 0 表示持球；旧字段名 BallIsHolding
