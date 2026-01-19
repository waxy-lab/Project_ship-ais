"""
测试评估框架 (Test Framework)

该模块负责批量运行测试场景，统计性能指标
提供测试工具函数和 Hypothesis 策略生成器
"""

__version__ = "0.1.0"

from .strategies import (
    # 基础策略
    latitude_strategy,
    longitude_strategy,
    heading_strategy,
    speed_strategy,
    rate_of_turn_strategy,
    mmsi_strategy,
    # 船舶状态策略
    ship_state_strategy,
    # 场景参数策略
    head_on_params_strategy,
    crossing_params_strategy,
    overtaking_params_strategy,
    emergency_params_strategy,
    # 环境配置策略
    environment_config_strategy,
    # 完整场景策略
    head_on_scenario_strategy,
    multi_ship_scenario_strategy,
)

from .test_helpers import (
    # 几何计算
    calculate_distance,
    calculate_bearing,
    calculate_relative_bearing,
    normalize_angle,
    angle_difference,
    # 船舶运动计算
    calculate_velocity_components,
    calculate_dcpa_tcpa,
    # 场景验证
    is_head_on_situation,
    is_crossing_situation,
    is_overtaking_situation,
    # 断言工具
    assert_valid_ship_state,
    assert_valid_scenario,
    assert_angle_in_range,
    assert_approximately_equal,
    # 性能指标
    calculate_collision_risk_index,
    calculate_path_efficiency,
)

__all__ = [
    # 策略
    'latitude_strategy',
    'longitude_strategy',
    'heading_strategy',
    'speed_strategy',
    'rate_of_turn_strategy',
    'mmsi_strategy',
    'ship_state_strategy',
    'head_on_params_strategy',
    'crossing_params_strategy',
    'overtaking_params_strategy',
    'emergency_params_strategy',
    'environment_config_strategy',
    'head_on_scenario_strategy',
    'multi_ship_scenario_strategy',
    # 工具函数
    'calculate_distance',
    'calculate_bearing',
    'calculate_relative_bearing',
    'normalize_angle',
    'angle_difference',
    'calculate_velocity_components',
    'calculate_dcpa_tcpa',
    'is_head_on_situation',
    'is_crossing_situation',
    'is_overtaking_situation',
    'assert_valid_ship_state',
    'assert_valid_scenario',
    'assert_angle_in_range',
    'assert_approximately_equal',
    'calculate_collision_risk_index',
    'calculate_path_efficiency',
]
