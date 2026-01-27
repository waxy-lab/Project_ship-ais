"""
避碰决策模块 (Collision Avoidance Module)

该模块负责实现基于COLREGS规则的智能避碰算法，包括：
- 碰撞风险评估 (Risk Assessment)
- COLREGS规则引擎 (Rules Engine)
- 路径规划 (Path Planning)
"""

__version__ = "0.1.0"

# 导出风险评估模块
from .risk_assessment import (
    # 常量
    SAFE_DISTANCE,
    SAFE_TIME,
    RISK_THRESHOLD_WARNING,
    RISK_THRESHOLD_DANGER,
    EMERGENCY_DCPA_THRESHOLD,
    EMERGENCY_TCPA_THRESHOLD,
    # 枚举
    RiskLevel,
    # 数据类
    RiskAssessment,
    CollisionRiskFactors,
    # 辅助函数
    degrees_to_nautical_miles,
    nautical_miles_to_degrees,
    normalize_angle,
    determine_risk_level,
)

# 导出规则引擎模块
from .rules_engine import (
    # 常量
    HEADING_DIFF_HEAD_ON_MIN,
    HEADING_DIFF_HEAD_ON_MAX,
    RELATIVE_BEARING_HEAD_ON_MIN,
    RELATIVE_BEARING_HEAD_ON_MAX,
    RELATIVE_BEARING_CROSSING_MIN,
    RELATIVE_BEARING_CROSSING_MAX,
    RELATIVE_BEARING_OVERTAKING,
    TURN_ANGLE_HEAD_ON,
    TURN_ANGLE_CROSSING_GIVE_WAY,
    TURN_ANGLE_OVERTAKING,
    SPEED_REDUCTION_FACTOR,
    # 枚举
    EncounterType,
    VesselRole,
    TurnDirection,
    ActionType,
    # 数据类
    AvoidanceAction,
    EncounterSituation,
    COLREGSRule,
    # 预定义规则
    COLREGS_RULES,
    # 辅助函数
    calculate_heading_difference,
    calculate_relative_bearing,
    is_crossing_from_starboard,
    get_applicable_rule,
)

__all__ = [
    # 风险评估 - 常量
    'SAFE_DISTANCE',
    'SAFE_TIME',
    'RISK_THRESHOLD_WARNING',
    'RISK_THRESHOLD_DANGER',
    'EMERGENCY_DCPA_THRESHOLD',
    'EMERGENCY_TCPA_THRESHOLD',
    # 风险评估 - 枚举
    'RiskLevel',
    # 风险评估 - 数据类
    'RiskAssessment',
    'CollisionRiskFactors',
    # 风险评估 - 辅助函数
    'degrees_to_nautical_miles',
    'nautical_miles_to_degrees',
    'normalize_angle',
    'determine_risk_level',
    # 规则引擎 - 常量
    'HEADING_DIFF_HEAD_ON_MIN',
    'HEADING_DIFF_HEAD_ON_MAX',
    'RELATIVE_BEARING_HEAD_ON_MIN',
    'RELATIVE_BEARING_HEAD_ON_MAX',
    'RELATIVE_BEARING_CROSSING_MIN',
    'RELATIVE_BEARING_CROSSING_MAX',
    'RELATIVE_BEARING_OVERTAKING',
    'TURN_ANGLE_HEAD_ON',
    'TURN_ANGLE_CROSSING_GIVE_WAY',
    'TURN_ANGLE_OVERTAKING',
    'SPEED_REDUCTION_FACTOR',
    # 规则引擎 - 枚举
    'EncounterType',
    'VesselRole',
    'TurnDirection',
    'ActionType',
    # 规则引擎 - 数据类
    'AvoidanceAction',
    'EncounterSituation',
    'COLREGSRule',
    # 规则引擎 - 预定义规则
    'COLREGS_RULES',
    # 规则引擎 - 辅助函数
    'calculate_heading_difference',
    'calculate_relative_bearing',
    'is_crossing_from_starboard',
    'get_applicable_rule',
]
