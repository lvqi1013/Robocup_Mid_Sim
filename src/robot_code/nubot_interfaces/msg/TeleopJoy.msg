# 手柄/键盘遥控命令。
# 由旧 Teleop_joy.msg 迁移而来，文件名按 ROS2 规范改为 TeleopJoy.msg。
float32 vx             # 遥控 x 方向速度；旧字段名 Vx
float32 vy             # 遥控 y 方向速度；旧字段名 Vy
float32 w              # 遥控角速度
int64 handle_enable    # 带球开关
float32 strength       # 射门力度
int64 shoot_pos        # 射门机构位置/模式；旧字段名 shootPos
int64 shoot_low_pos    # 低射机构位置/模式；旧字段名 shootLowPos
