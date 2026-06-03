# 单个机器人状态信息。
std_msgs/Header header  # 时间戳和坐标系信息
int32 agent_id          # 机器人编号；旧字段名 AgentID
int32 target_num1       # 策略目标编号 1
int32 target_num2       # 策略目标编号 2
int32 target_num3       # 策略目标编号 3
int32 target_num4       # 策略目标编号 4
int32 staticpass_num    # 静态传球编号；旧字段名 staticpassNum
int32 staticcatch_num   # 静态接球编号；旧字段名 staticcatchNum
Point2D pos             # 机器人二维位置
Angle heading           # 机器人朝向
float32 vrot            # 机器人旋转速度
Point2D vtrans          # 机器人平移速度
bool is_kick            # 是否正在射门；旧字段名 iskick
bool is_valid           # 信息是否有效；旧字段名 isvalid
bool is_stuck           # 机器人是否卡住；旧字段名 isstuck
bool is_dribble         # 是否正在带球；旧字段名 isdribble
int8 current_role       # 当前角色编号，旧类型 char
float32 role_time       # 当前角色持续时间
Point2D target          # 当前策略目标点
