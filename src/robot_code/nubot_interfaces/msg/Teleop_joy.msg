# 用于调试和演示，通过游戏手柄直接控制机器人。
float32 vx             # 摇杆指定的 X 速度
float32 vy             # 摇杆指定的 Y 速度
float32 w              # 摇杆指定的角速度
int8   handle_enable  # 摇杆触发带球
float32 strength       # 摇杆触发射门力量
int8   shoot_pos       # 摇杆触发射门模式
int8   shoot_low_pos    # 摇杆触发低射门（推射）