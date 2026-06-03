# Coach 侧世界模型信息。
std_msgs/Header header             # 时间戳和坐标系信息
ObstaclesInfo[] obstacleinfo       # 障碍物信息列表，保留旧字段名语义
ObstaclesInfo oppinfo              # 对方机器人/障碍物信息
RobotInfo[] robotinfo              # 我方机器人信息列表
BallInfo[] ballinfo                # 球信息列表
PassCommands pass_cmd              # 当前传球协作命令
