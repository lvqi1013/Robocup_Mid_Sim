# Coach/refbox 下发的比赛控制信息。
std_msgs/Header header  # 时间戳和坐标系信息
int8 match_mode         # 比赛模式，旧类型 char，旧字段名 MatchMode
int8 match_type         # 比赛类型，旧类型 char，旧字段名 MatchType
int8 test_mode          # 测试模式，旧类型 char，旧字段名 TestMode
Point2D point_a         # 裁判/coach 命令相关点 A
Point2D point_b         # 裁判/coach 命令相关点 B
int16 angle_a           # 与点 A 相关的角度
int16 angle_b           # 与点 B 相关的角度
int8 id_a               # 相关机器人编号 A，旧类型 char
int8 id_b               # 相关机器人编号 B，旧类型 char
int8 kickforce          # 开球/射门力度，旧类型 char
