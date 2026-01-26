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

__all__ = [
    # 常量
    'SAFE_DISTANCE',
    'SAFE_TIME',
    'RISK_THRESHOLD_WARNING',
    'RISK_THRESHOLD_DANGER',
    'EMERGENCY_DCPA_THRESHOLD',
    'EMERGENCY_TCPA_THRESHOLD',
    # 枚举
    'RiskLevel',
    # 数据类
    'RiskAssessment',
    'CollisionRiskFactors',
    # 辅助函数
    'degrees_to_nautical_miles',
    'nautical_miles_to_degrees',
    'normalize_angle',
    'determine_risk_level',
]
