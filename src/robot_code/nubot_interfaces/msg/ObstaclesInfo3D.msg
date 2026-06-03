# 三维障碍物信息。
std_msgs/Header header  # 时间戳和坐标系信息
Point3D[] pos           # 障碍物三维坐标列表
Point3D pos_min         # 障碍物集合或目标区域的最小边界点
bool pos_known_3d       # 三维位置是否可信
bool pos_known_2d       # 二维投影位置是否可信
