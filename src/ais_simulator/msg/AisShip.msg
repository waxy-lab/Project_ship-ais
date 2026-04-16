# 用于定义单个AIS目标船的状态

std_msgs/Header header

uint32 mmsi         # 船的MMSI (唯一标识)
float64 latitude    # 纬度
float64 longitude   # 经度
float32 heading     # 航向 (0-359.9 度)
float32 sog         # 航速 (Speed Over Ground, 节)
float32 rot         # 转向率 (Rate of Turn, 度/分钟)
float32 length      # 船长 (米)
float32 width       # 船宽 (米)
string role         # 船舶角色: own_ship / target_ship / other
bool collision_avoidance_enabled  # 是否启用避碰控制
