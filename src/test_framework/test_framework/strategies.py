"""
Hypothesis 策略生成器
用于基于属性的测试（Property-Based Testing）

提供各种测试数据生成策略，包括：
- 船舶状态数据
- 场景参数
- 环境配置
"""

from hypothesis import strategies as st
from hypothesis.strategies import composite
import math


# ============================================================================
# 基础数据策略
# ============================================================================

@composite
def latitude_strategy(draw):
    """生成有效的纬度值 (-90 到 90 度)"""
    return draw(st.floats(
        min_value=-90.0, 
        max_value=90.0, 
        allow_nan=False, 
        allow_infinity=False
    ))


@composite
def longitude_strategy(draw):
    """生成有效的经度值 (-180 到 180 度)"""
    return draw(st.floats(
        min_value=-180.0, 
        max_value=180.0, 
        allow_nan=False, 
        allow_infinity=False
    ))


@composite
def heading_strategy(draw):
    """生成有效的航向值 (0 到 360 度，不包括360)"""
    return draw(st.floats(
        min_value=0.0, 
        max_value=359.999999, 
        allow_nan=False, 
        allow_infinity=False
    ))


@composite
def speed_strategy(draw, min_speed=0.0, max_speed=30.0):
    """
    生成有效的船舶速度值 (节)
    
    Args:
        min_speed: 最小速度，默认0节
        max_speed: 最大速度，默认30节
    """
    return draw(st.floats(
        min_value=min_speed, 
        max_value=max_speed, 
        allow_nan=False, 
        allow_infinity=False
    ))


@composite
def rate_of_turn_strategy(draw):
    """生成有效的转向率 (-10 到 10 度/分钟)"""
    return draw(st.floats(
        min_value=-10.0, 
        max_value=10.0, 
        allow_nan=False, 
        allow_infinity=False
    ))


@composite
def mmsi_strategy(draw):
    """生成有效的MMSI号码 (9位数字)"""
    return draw(st.integers(min_value=100000000, max_value=999999999))


# ============================================================================
# 船舶状态策略
# ============================================================================

@composite
def ship_state_strategy(draw, 
                       lat_range=None, 
                       lon_range=None,
                       speed_range=None,
                       heading_range=None):
    """
    生成船舶状态数据
    
    Args:
        lat_range: 纬度范围 (min, max)，默认 (-90, 90)
        lon_range: 经度范围 (min, max)，默认 (-180, 180)
        speed_range: 速度范围 (min, max)，默认 (0, 30)
        heading_range: 航向范围 (min, max)，默认 (0, 360)
    
    Returns:
        dict: 船舶状态字典
    """
    # 设置默认范围
    lat_min, lat_max = lat_range if lat_range else (-90.0, 90.0)
    lon_min, lon_max = lon_range if lon_range else (-180.0, 180.0)
    speed_min, speed_max = speed_range if speed_range else (0.0, 30.0)
    heading_min, heading_max = heading_range if heading_range else (0.0, 359.999999)
    
    return {
        'mmsi': draw(mmsi_strategy()),
        'latitude': draw(st.floats(
            min_value=lat_min, 
            max_value=lat_max, 
            allow_nan=False, 
            allow_infinity=False
        )),
        'longitude': draw(st.floats(
            min_value=lon_min, 
            max_value=lon_max, 
            allow_nan=False, 
            allow_infinity=False
        )),
        'heading': draw(st.floats(
            min_value=heading_min, 
            max_value=heading_max, 
            allow_nan=False, 
            allow_infinity=False
        )),
        'sog': draw(st.floats(
            min_value=speed_min, 
            max_value=speed_max, 
            allow_nan=False, 
            allow_infinity=False
        )),
        'rot': draw(rate_of_turn_strategy()),
        'course': draw(heading_strategy()),
        'navigation_status': draw(st.sampled_from([
            'under way using engine', 
            'at anchor', 
            'not under command', 
            'restricted maneuverability'
        ]))
    }


# ============================================================================
# 场景参数策略
# ============================================================================

@composite
def head_on_params_strategy(draw):
    """
    生成对遇场景参数
    
    Returns:
        dict: 对遇场景参数
    """
    return {
        'distance': draw(st.floats(
            min_value=0.5, 
            max_value=10.0, 
            allow_nan=False, 
            allow_infinity=False
        )),
        'speed1': draw(st.floats(
            min_value=5.0, 
            max_value=20.0, 
            allow_nan=False, 
            allow_infinity=False
        )),
        'speed2': draw(st.floats(
            min_value=5.0, 
            max_value=20.0, 
            allow_nan=False, 
            allow_infinity=False
        )),
        'base_lat': draw(st.floats(
            min_value=20.0, 
            max_value=50.0, 
            allow_nan=False, 
            allow_infinity=False
        )),
        'base_lon': draw(st.floats(
            min_value=100.0, 
            max_value=140.0, 
            allow_nan=False, 
            allow_infinity=False
        ))
    }


@composite
def crossing_params_strategy(draw):
    """
    生成交叉相遇场景参数
    
    Returns:
        dict: 交叉场景参数
    """
    return {
        'distance': draw(st.floats(
            min_value=0.5, 
            max_value=10.0, 
            allow_nan=False, 
            allow_infinity=False
        )),
        'crossing_angle': draw(st.floats(
            min_value=30.0, 
            max_value=150.0, 
            allow_nan=False, 
            allow_infinity=False
        )),
        'speed1': draw(st.floats(
            min_value=5.0, 
            max_value=20.0, 
            allow_nan=False, 
            allow_infinity=False
        )),
        'speed2': draw(st.floats(
            min_value=5.0, 
            max_value=20.0, 
            allow_nan=False, 
            allow_infinity=False
        )),
        'base_lat': draw(st.floats(
            min_value=20.0, 
            max_value=50.0, 
            allow_nan=False, 
            allow_infinity=False
        )),
        'base_lon': draw(st.floats(
            min_value=100.0, 
            max_value=140.0, 
            allow_nan=False, 
            allow_infinity=False
        ))
    }


@composite
def overtaking_params_strategy(draw):
    """
    生成追越场景参数
    
    Returns:
        dict: 追越场景参数
    """
    speed1 = draw(st.floats(
        min_value=5.0, 
        max_value=15.0, 
        allow_nan=False, 
        allow_infinity=False
    ))
    # 追越船速度必须大于被追越船
    speed2 = draw(st.floats(
        min_value=speed1 + 2.0, 
        max_value=25.0, 
        allow_nan=False, 
        allow_infinity=False
    ))
    
    return {
        'distance': draw(st.floats(
            min_value=0.5, 
            max_value=5.0, 
            allow_nan=False, 
            allow_infinity=False
        )),
        'speed1': speed1,  # 被追越船
        'speed2': speed2,  # 追越船
        'overtaking_angle': draw(st.floats(
            min_value=150.0, 
            max_value=210.0, 
            allow_nan=False, 
            allow_infinity=False
        )),
        'base_lat': draw(st.floats(
            min_value=20.0, 
            max_value=50.0, 
            allow_nan=False, 
            allow_infinity=False
        )),
        'base_lon': draw(st.floats(
            min_value=100.0, 
            max_value=140.0, 
            allow_nan=False, 
            allow_infinity=False
        ))
    }


@composite
def emergency_params_strategy(draw):
    """
    生成紧急避让场景参数（DCPA < 0.5海里，TCPA < 5分钟）
    
    Returns:
        dict: 紧急场景参数
    """
    return {
        'dcpa': draw(st.floats(
            min_value=0.1, 
            max_value=0.5, 
            allow_nan=False, 
            allow_infinity=False
        )),
        'tcpa': draw(st.floats(
            min_value=60.0, 
            max_value=300.0, 
            allow_nan=False, 
            allow_infinity=False
        )),  # 秒
        'speed1': draw(st.floats(
            min_value=8.0, 
            max_value=20.0, 
            allow_nan=False, 
            allow_infinity=False
        )),
        'speed2': draw(st.floats(
            min_value=8.0, 
            max_value=20.0, 
            allow_nan=False, 
            allow_infinity=False
        )),
        'base_lat': draw(st.floats(
            min_value=20.0, 
            max_value=50.0, 
            allow_nan=False, 
            allow_infinity=False
        )),
        'base_lon': draw(st.floats(
            min_value=100.0, 
            max_value=140.0, 
            allow_nan=False, 
            allow_infinity=False
        ))
    }


# ============================================================================
# 环境配置策略
# ============================================================================

@composite
def environment_config_strategy(draw):
    """
    生成环境配置参数
    
    Returns:
        dict: 环境配置
    """
    return {
        'weather_condition': draw(st.sampled_from(['calm', 'moderate', 'rough'])),
        'visibility': draw(st.sampled_from(['good', 'moderate', 'poor'])),
        'water_area_type': draw(st.sampled_from(['open', 'restricted', 'narrow'])),
        'wind_speed': draw(st.floats(
            min_value=0.0, 
            max_value=30.0, 
            allow_nan=False, 
            allow_infinity=False
        )),
        'wind_direction': draw(heading_strategy()),
        'current_speed': draw(st.floats(
            min_value=0.0, 
            max_value=5.0, 
            allow_nan=False, 
            allow_infinity=False
        )),
        'current_direction': draw(heading_strategy())
    }


# ============================================================================
# 完整场景策略
# ============================================================================

@composite
def head_on_scenario_strategy(draw):
    """
    生成完整的对遇场景配置
    
    Returns:
        dict: 完整场景配置
    """
    params = draw(head_on_params_strategy())
    
    # 生成两艘相向航行的船舶
    ship1 = {
        'mmsi': 123456789,
        'latitude': params['base_lat'],
        'longitude': params['base_lon'],
        'heading': 90.0,
        'sog': params['speed1'],
        'rot': 0.0,
        'course': 90.0,
        'navigation_status': 'under way using engine'
    }
    
    ship2 = {
        'mmsi': 987654321,
        'latitude': params['base_lat'],
        'longitude': params['base_lon'] + params['distance'] / 60.0,  # 转换为度
        'heading': 270.0,
        'sog': params['speed2'],
        'rot': 0.0,
        'course': 270.0,
        'navigation_status': 'under way using engine'
    }
    
    return {
        'scenario_type': 'head_on',
        'ships': [ship1, ship2],
        'environment': draw(environment_config_strategy()),
        'duration': draw(st.floats(
            min_value=300.0, 
            max_value=1800.0, 
            allow_nan=False, 
            allow_infinity=False
        ))
    }


@composite
def multi_ship_scenario_strategy(draw, num_ships=None):
    """
    生成多船场景配置
    
    Args:
        num_ships: 船舶数量，默认随机3-6艘
    
    Returns:
        dict: 多船场景配置
    """
    if num_ships is None:
        num_ships = draw(st.integers(min_value=3, max_value=6))
    
    base_lat = draw(st.floats(
        min_value=20.0, 
        max_value=50.0, 
        allow_nan=False, 
        allow_infinity=False
    ))
    base_lon = draw(st.floats(
        min_value=100.0, 
        max_value=140.0, 
        allow_nan=False, 
        allow_infinity=False
    ))
    
    ships = []
    for i in range(num_ships):
        ship = draw(ship_state_strategy(
            lat_range=(base_lat - 0.1, base_lat + 0.1),
            lon_range=(base_lon - 0.1, base_lon + 0.1),
            speed_range=(5.0, 20.0)
        ))
        ship['mmsi'] = 100000000 + i
        ships.append(ship)
    
    return {
        'scenario_type': 'multi_ship',
        'ships': ships,
        'environment': draw(environment_config_strategy()),
        'duration': draw(st.floats(
            min_value=300.0, 
            max_value=1800.0, 
            allow_nan=False, 
            allow_infinity=False
        ))
    }
