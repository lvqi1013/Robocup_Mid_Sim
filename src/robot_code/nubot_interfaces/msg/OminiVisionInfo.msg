# 全局/全向视觉信息。
# 注意：旧项目文件名为 OminiVisionInfo.msg，疑似 Omni 拼写错误；为减少迁移破坏，类型名保持 OminiVisionInfo。
std_msgs/Header header        # 时间戳和坐标系信息
BallInfo ballinfo             # 球信息
ObstaclesInfo obstacleinfo    # 障碍物信息
RobotInfo[] robotinfo         # 机器人信息列表
