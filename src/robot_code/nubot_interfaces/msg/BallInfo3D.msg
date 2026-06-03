# 三维球信息。
std_msgs/Header header  # 时间戳和坐标系信息
Point3D pos             # 球的三维位置
Point3D velocity        # 球的三维速度
bool pos_known_3d       # 三维位置是否可信
bool pos_known_2d       # 投影到二维平面的位置是否可信
bool velocity_known     # 速度是否可信
