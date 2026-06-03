# 单机器人使用的世界模型信息。
std_msgs/Header header        # 时间戳和坐标系信息
ObstaclesInfo obstacleinfo    # 障碍物信息
ObstaclesInfo oppinfo         # 对方机器人/障碍物信息
RobotInfo[] robotinfo         # 队友或相关机器人信息
BallInfo[] ballinfo           # 球信息列表
CoachInfo coachinfo           # coach/refbox 比赛控制信息
PassCommands pass_cmd         # 当前传球协作命令
