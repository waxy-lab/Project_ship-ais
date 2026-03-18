# 避碰控制指令消息
# 由避碰决策节点发布，AIS模拟器订阅并执行
# Requirements: 5.5

std_msgs/Header header

uint32 own_ship_mmsi      # 本船MMSI
float64 timestamp         # 指令生成时间戳（Unix时间，秒）

# 目标航向（度，0-359.9），NaN表示不调整
float32 target_heading

# 目标航速（节），NaN表示不调整
float32 target_speed

# 转向率（度/分钟），正值右转，负值左转，NaN表示自动
float32 turn_rate

# 指令持续时间（秒），0表示持续到下一条指令
float32 duration

# 指令类型字符串
# 取值: "course_change", "speed_change", "course_and_speed_change",
#       "maintain", "return_to_original_course"
string command_type

# 指令来源描述（用于日志和调试）
string reason
