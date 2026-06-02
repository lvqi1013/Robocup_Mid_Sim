Point2D target # 目标点坐标 (m)
float32 target_ori # 目标朝向（弧度）
float32 target_w # 目标角速度 (rad/s)
Point2D target_vel  # 目标速度 (m/s)
float32 max_vel # 最大线速度 (m/s)
float32 max_w # 最大角速度 (rad/s)
Point2D robot_pos # 机器人当前位置（回传）
Point2D robot_vel  # 机器人当前速度（回传）
float32 robot_ori # 机器人当前朝向（回传）
float32 robot_w # 机器人当前角速度（回传）
uint8    move_action  # 平移动作类型（枚举：No_Action/CatchBall/MoveWithBall/TurnForShoot/...）
uint8    rotate_action  # 旋转动作类型（同上）
int32   rotate_mode  # 旋转模式：0=绝对角度, 1=相对角度
int64   handle_enable # 带球使能：1=允许带球, 0=禁止带球
float32 strength  # 射门力量（0~15，对应踢球速度
int64   shoot_pos  # 射门模式：RUN=平射, FLY=挑射