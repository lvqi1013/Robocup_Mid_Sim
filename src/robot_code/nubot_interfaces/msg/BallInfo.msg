# 二维球信息。
std_msgs/Header header    # 时间戳和坐标系信息
int32 ball_info_state     # 球状态编号；旧字段名 ballinfostate
Point2D pos               # 球在二维平面中的笛卡尔位置
PPoint real_pos           # 球相对机器人或观测源的极坐标位置
Point2D velocity          # 球在二维平面中的速度
bool pos_known            # 球位置是否可信
bool velocity_known       # 球速度是否可信
