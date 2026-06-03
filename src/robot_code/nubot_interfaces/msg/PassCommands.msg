# 传球协作命令。
uint32 pass_id          # 传球机器人编号
uint32 catch_id         # 接球机器人编号
Point2D pass_pt         # 传球点
Point2D catch_pt        # 接球点
bool is_passout         # 是否正在执行传球输出
bool is_dynamic_pass    # 是否为动态传球
bool is_static_pass     # 是否为静态传球
bool is_valid           # 该传球命令是否有效
