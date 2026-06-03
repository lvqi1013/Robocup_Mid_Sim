# 单个机器人策略状态。
std_msgs/Header header  # 时间戳和坐标系信息
int32 agent_id          # 机器人编号；旧字段名 AgentID
int32 target_num1       # 策略目标编号 1
int32 target_num2       # 策略目标编号 2
int32 target_num3       # 策略目标编号 3
int32 target_num4       # 策略目标编号 4
int32 staticpass_num    # 静态传球编号；旧字段名 staticpassNum
int32 staticcatch_num   # 静态接球编号；旧字段名 staticcatchNum
uint32 role             # 当前角色编号
uint32 action           # 当前动作编号
bool is_dribble         # 是否正在带球
bool is_kickoff         # 是否处于开球相关状态
float32 role_time       # 当前角色持续时间
PassCommands pass_cmd   # 当前传球协作命令
