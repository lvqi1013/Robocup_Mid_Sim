# 通用观测对象信息。
# 由旧 object_info.msg 迁移而来，文件名按 ROS2 规范改为 ObjectInfo.msg。
std_msgs/Header header           # 时间戳和坐标系信息
geometry_msgs/Point[] ball_pos   # 检测到的球位置列表
geometry_msgs/Point[] obs_pos    # 检测到的障碍物位置列表
bool ball_know                   # 是否识别到球；保留旧字段拼写语义
bool obs_know                    # 是否识别到障碍物
