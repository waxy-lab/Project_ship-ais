"""
测试工具函数
提供常用的测试辅助函数和断言工具
"""

import math
from typing import Dict, List, Tuple, Any


# ============================================================================
# 几何计算工具
# ============================================================================

def calculate_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    计算两点之间的距离（海里）
    使用 Haversine 公式
    
    Args:
        lat1, lon1: 第一个点的纬度和经度（度）
        lat2, lon2: 第二个点的纬度和经度（度）
    
    Returns:
        float: 距离（海里）
    """
    # 地球半径（海里）
    R = 3440.065
    
    # 转换为弧度
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)
    
    # Haversine 公式
    dlat = lat2_rad - lat1_rad
    dlon = lon2_rad - lon1_rad
    
    a = math.sin(dlat/2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon/2)**2
    c = 2 * math.asin(math.sqrt(a))
    
    return R * c


def calculate_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    计算从点1到点2的方位角（度）
    
    Args:
        lat1, lon1: 起点的纬度和经度（度）
        lat2, lon2: 终点的纬度和经度（度）
    
    Returns:
        float: 方位角（0-360度，0度为正北）
    """
    # 转换为弧度
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)
    
    dlon = lon2_rad - lon1_rad
    
    y = math.sin(dlon) * math.cos(lat2_rad)
    x = math.cos(lat1_rad) * math.sin(lat2_rad) - \
        math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon)
    
    bearing_rad = math.atan2(y, x)
    bearing_deg = math.degrees(bearing_rad)
    
    # 转换为0-360度
    return (bearing_deg + 360) % 360


def calculate_relative_bearing(own_heading: float, target_bearing: float) -> float:
    """
    计算相对方位角
    
    Args:
        own_heading: 本船航向（度）
        target_bearing: 目标方位（度）
    
    Returns:
        float: 相对方位角（-180到180度，正值表示右舷）
    """
    relative = target_bearing - own_heading
    
    # 归一化到-180到180度
    if relative > 180:
        relative -= 360
    elif relative < -180:
        relative += 360
    
    return relative


def normalize_angle(angle: float) -> float:
    """
    将角度归一化到0-360度范围
    
    Args:
        angle: 角度（度）
    
    Returns:
        float: 归一化后的角度（0-360度）
    """
    return angle % 360


def angle_difference(angle1: float, angle2: float) -> float:
    """
    计算两个角度之间的最小差值
    
    Args:
        angle1, angle2: 角度（度）
    
    Returns:
        float: 角度差（0-180度）
    """
    diff = abs(angle1 - angle2)
    if diff > 180:
        diff = 360 - diff
    return diff


# ============================================================================
# 船舶运动计算
# ============================================================================

def calculate_velocity_components(speed: float, heading: float) -> Tuple[float, float]:
    """
    计算速度的东向和北向分量
    
    Args:
        speed: 速度（节）
        heading: 航向（度，0度为正北）
    
    Returns:
        Tuple[float, float]: (东向速度, 北向速度)
    """
    heading_rad = math.radians(heading)
    vx = speed * math.sin(heading_rad)  # 东向
    vy = speed * math.cos(heading_rad)  # 北向
    return vx, vy


def calculate_dcpa_tcpa(ship1: Dict, ship2: Dict) -> Tuple[float, float]:
    """
    计算DCPA（最近会遇距离）和TCPA（到达最近点时间）
    
    Args:
        ship1: 船舶1状态字典（包含latitude, longitude, speed, heading）
        ship2: 船舶2状态字典
    
    Returns:
        Tuple[float, float]: (DCPA海里, TCPA秒)
    """
    # 当前距离
    current_distance = calculate_distance(
        ship1['latitude'], ship1['longitude'],
        ship2['latitude'], ship2['longitude']
    )
    
    # 速度分量
    vx1, vy1 = calculate_velocity_components(ship1['speed'], ship1['heading'])
    vx2, vy2 = calculate_velocity_components(ship2['speed'], ship2['heading'])
    
    # 相对速度
    dvx = vx2 - vx1
    dvy = vy2 - vy1
    
    # 相对位置（转换为海里）
    dx = (ship2['longitude'] - ship1['longitude']) * 60 * math.cos(math.radians(ship1['latitude']))
    dy = (ship2['latitude'] - ship1['latitude']) * 60
    
    # 相对速度的平方
    dv_squared = dvx**2 + dvy**2
    
    if dv_squared < 1e-6:  # 相对速度接近0
        return current_distance, -1.0
    
    # TCPA计算（小时）
    tcpa_hours = -(dx * dvx + dy * dvy) / dv_squared
    
    if tcpa_hours < 0:  # 已经过了最近点
        return current_distance, -1.0
    
    # DCPA计算
    dcpa = math.sqrt((dx + dvx * tcpa_hours)**2 + (dy + dvy * tcpa_hours)**2)
    
    # 转换TCPA为秒
    tcpa_seconds = tcpa_hours * 3600
    
    return dcpa, tcpa_seconds


# ============================================================================
# 场景验证工具
# ============================================================================

def is_head_on_situation(ship1: Dict, ship2: Dict, 
                        heading_tolerance: float = 20.0,
                        bearing_tolerance: float = 10.0) -> bool:
    """
    判断是否为对遇局面
    
    Args:
        ship1, ship2: 船舶状态字典
        heading_tolerance: 航向差容差（度）
        bearing_tolerance: 相对方位容差（度）
    
    Returns:
        bool: 是否为对遇局面
    """
    # 计算航向差
    heading_diff = angle_difference(ship1['heading'], ship2['heading'])
    
    # 计算相对方位
    bearing = calculate_bearing(
        ship1['latitude'], ship1['longitude'],
        ship2['latitude'], ship2['longitude']
    )
    relative_bearing = calculate_relative_bearing(ship1['heading'], bearing)
    
    # 对遇判定：航向差接近180度，相对方位接近0度
    is_opposite_heading = abs(heading_diff - 180) < heading_tolerance
    is_ahead = abs(relative_bearing) < bearing_tolerance
    
    return is_opposite_heading and is_ahead


def is_crossing_situation(ship1: Dict, ship2: Dict,
                         min_angle: float = 5.0,
                         max_angle: float = 112.5) -> bool:
    """
    判断是否为交叉相遇局面
    
    Args:
        ship1, ship2: 船舶状态字典
        min_angle: 最小相对方位角（度）
        max_angle: 最大相对方位角（度）
    
    Returns:
        bool: 是否为交叉相遇局面
    """
    bearing = calculate_bearing(
        ship1['latitude'], ship1['longitude'],
        ship2['latitude'], ship2['longitude']
    )
    relative_bearing = abs(calculate_relative_bearing(ship1['heading'], bearing))
    
    return min_angle <= relative_bearing <= max_angle


def is_overtaking_situation(ship1: Dict, ship2: Dict,
                           angle_range: float = 22.5) -> bool:
    """
    判断ship2是否在追越ship1
    
    Args:
        ship1: 被追越船状态
        ship2: 追越船状态
        angle_range: 后方扇形区域角度（度）
    
    Returns:
        bool: 是否为追越局面
    """
    # 计算相对方位
    bearing = calculate_bearing(
        ship1['latitude'], ship1['longitude'],
        ship2['latitude'], ship2['longitude']
    )
    relative_bearing = abs(calculate_relative_bearing(ship1['heading'], bearing))
    
    # 追越判定：从后方22.5度扇形区域接近，且速度更快
    is_from_behind = relative_bearing > (180 - angle_range)
    is_faster = ship2['speed'] > ship1['speed']
    
    return is_from_behind and is_faster


# ============================================================================
# 断言工具
# ============================================================================

def assert_valid_ship_state(ship: Dict):
    """
    断言船舶状态数据有效
    
    Args:
        ship: 船舶状态字典
    
    Raises:
        AssertionError: 如果数据无效
    """
    assert 'mmsi' in ship, "缺少MMSI字段"
    assert 'latitude' in ship, "缺少latitude字段"
    assert 'longitude' in ship, "缺少longitude字段"
    assert 'heading' in ship, "缺少heading字段"
    assert 'speed' in ship, "缺少speed字段"
    
    assert -90 <= ship['latitude'] <= 90, f"纬度超出范围: {ship['latitude']}"
    assert -180 <= ship['longitude'] <= 180, f"经度超出范围: {ship['longitude']}"
    assert 0 <= ship['heading'] <= 360, f"航向超出范围: {ship['heading']}"
    assert ship['speed'] >= 0, f"速度不能为负: {ship['speed']}"


def assert_valid_scenario(scenario: Dict):
    """
    断言场景配置有效
    
    Args:
        scenario: 场景配置字典
    
    Raises:
        AssertionError: 如果配置无效
    """
    assert 'scenario_type' in scenario, "缺少scenario_type字段"
    assert 'ships' in scenario, "缺少ships字段"
    assert len(scenario['ships']) >= 2, "至少需要2艘船舶"
    
    for ship in scenario['ships']:
        assert_valid_ship_state(ship)


def assert_angle_in_range(angle: float, min_angle: float, max_angle: float, 
                         message: str = ""):
    """
    断言角度在指定范围内
    
    Args:
        angle: 角度值
        min_angle: 最小角度
        max_angle: 最大角度
        message: 错误消息
    """
    normalized = normalize_angle(angle)
    assert min_angle <= normalized <= max_angle, \
        f"{message} 角度 {normalized} 不在范围 [{min_angle}, {max_angle}] 内"


def assert_approximately_equal(value1: float, value2: float, 
                              tolerance: float = 0.01,
                              message: str = ""):
    """
    断言两个浮点数近似相等
    
    Args:
        value1, value2: 要比较的值
        tolerance: 容差（相对误差）
        message: 错误消息
    """
    if abs(value1) < 1e-10 and abs(value2) < 1e-10:
        return  # 都接近0
    
    relative_error = abs(value1 - value2) / max(abs(value1), abs(value2))
    assert relative_error <= tolerance, \
        f"{message} {value1} 和 {value2} 差异过大（相对误差: {relative_error:.4f}）"


# ============================================================================
# 性能指标计算
# ============================================================================

def calculate_collision_risk_index(dcpa: float, tcpa: float,
                                  safe_distance: float = 0.5,
                                  safe_time: float = 300.0) -> float:
    """
    计算碰撞风险指数（CRI）
    
    Args:
        dcpa: 最近会遇距离（海里）
        tcpa: 到达最近点时间（秒）
        safe_distance: 安全距离（海里）
        safe_time: 安全时间（秒）
    
    Returns:
        float: 碰撞风险指数（0-1，值越大风险越高）
    """
    if tcpa < 0:
        return 0.0  # 已经过了最近点
    
    # 距离因子
    distance_factor = 1.0 / (1.0 + dcpa / safe_distance)
    
    # 时间因子
    time_factor = 1.0 / (1.0 + tcpa / safe_time)
    
    # 综合风险指数
    cri = 0.6 * distance_factor + 0.4 * time_factor
    
    return min(cri, 1.0)


def calculate_path_efficiency(original_distance: float, 
                             actual_distance: float) -> float:
    """
    计算路径效率（航程增加百分比）
    
    Args:
        original_distance: 原始直线距离
        actual_distance: 实际航行距离
    
    Returns:
        float: 航程增加百分比
    """
    if original_distance <= 0:
        return 0.0
    
    return ((actual_distance - original_distance) / original_distance) * 100
