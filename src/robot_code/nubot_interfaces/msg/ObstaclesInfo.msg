# 二维障碍物集合。
std_msgs/Header header  # 时间戳和坐标系信息
Point2D[] pos           # 障碍物二维笛卡尔坐标列表
PPoint[] polar_pos      # 障碍物极坐标列表，通常相对当前机器人
