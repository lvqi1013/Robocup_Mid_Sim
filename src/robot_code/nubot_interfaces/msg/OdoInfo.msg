# 里程计/底盘状态信息。
std_msgs/Header header  # 时间戳和坐标系信息
float32 vx              # 机器人本体 x 方向速度；旧字段名 Vx
float32 vy              # 机器人本体 y 方向速度；旧字段名 Vy
float32 w               # 绕 z 轴角速度
bool robot_stuck        # 机器人是否被判定卡住；旧字段名 RobotStuck
bool power_state        # 电源或执行系统是否正常；旧字段名 PowerState
