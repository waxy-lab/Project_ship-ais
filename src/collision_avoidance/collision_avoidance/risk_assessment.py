"""
碰撞风险评估模块 (Risk Assessment Module)

该模块负责计算碰撞风险指标，包括：
- DCPA (Distance at Closest Point of Approach) - 最近会遇距离
- TCPA (Time to Closest Point of Approach) - 到达最近会遇点时间
- CRI (Collision Risk Index) - 碰撞风险指数

Requirements: 4.1-4.4
"""

from dataclasses import dataclass
from enum import Enum
from typing import Tuple, Optional
import math


# ============================================================================
# 常量定义
# ============================================================================

# 安全距离阈值（海里）
SAFE_DISTANCE = 2.0  # 安全距离：2海里

# 安全时间阈值（分钟）
SAFE_TIME = 10.0  # 安全时间：10分钟

# 风险等级阈值
RISK_THRESHOLD_WARNING = 0.5  # 预警阈值：CRI >= 0.5 触发预警
RISK_THRESHOLD_DANGER = 0.7   # 危险阈值：CRI >= 0.7 触发自动避让

# 紧急场景阈值（用于场景生成验证）
EMERGENCY_DCPA_THRESHOLD = 0.5  # 紧急场景DCPA阈值：0.5海里
EMERGENCY_TCPA_THRESHOLD = 5.0  # 紧急场景TCPA阈值：5分钟

# 单位转换常量
KNOTS_TO_MS = 0.514444  # 节转米/秒
NM_TO_METERS = 1852.0   # 海里转米
DEGREES_TO_NM = 60.0    # 纬度/经度度数转海里（近似）

# 方位角范围常量（用于相遇类型判定）
BEARING_HEAD_ON_MIN = -10.0   # 对遇方位角最小值
BEARING_HEAD_ON_MAX = 10.0    # 对遇方位角最大值
BEARING_CROSSING_MIN = 5.0    # 交叉相遇方位角最小值
BEARING_CROSSING_MAX = 112.5  # 交叉相遇方位角最大值
BEARING_OVERTAKING = 112.5    # 追越方位角阈值（后方22.5度扇形）


# ============================================================================
# 枚举类型定义
# ============================================================================

class RiskLevel(Enum):
    """风险等级枚举
    
    根据CRI值划分风险等级：
    - SAFE: 安全（CRI < 0.5）
    - WARNING: 预警（0.5 <= CRI < 0.7）
    - DANGER: 危险（CRI >= 0.7）
    """
    SAFE = "safe"           # 安全
    WARNING = "warning"     # 预警
    DANGER = "danger"       # 危险


# ============================================================================
# 数据类定义
# ============================================================================

@dataclass
class RiskAssessment:
    """碰撞风险评估结果
    
    包含DCPA、TCPA、CRI等风险指标
    
    Attributes:
        dcpa: 最近会遇距离（海里）
        tcpa: 到达最近会遇点时间（分钟）
        cri: 碰撞风险指数（0-1）
        risk_level: 风险等级
        relative_bearing: 相对方位（度）
        distance: 当前距离（海里）
        speed_ratio: 速度比（目标船速度/本船速度）
    """
    dcpa: float                      # 最近会遇距离（海里）
    tcpa: float                      # 到达最近会遇点时间（分钟）
    cri: float                       # 碰撞风险指数（0-1）
    risk_level: RiskLevel            # 风险等级
    relative_bearing: float          # 相对方位（度）
    distance: float                  # 当前距离（海里）
    speed_ratio: float               # 速度比
    
    def __post_init__(self):
        """数据验证"""
        # 验证DCPA非负
        if self.dcpa < 0:
            raise ValueError(f"DCPA不能为负数: {self.dcpa}")
        
        # 验证CRI范围
        if not (0 <= self.cri <= 1):
            raise ValueError(f"CRI必须在0到1之间: {self.cri}")
        
        # 验证相对方位范围
        if not (-180 <= self.relative_bearing <= 180):
            raise ValueError(f"相对方位必须在-180到180度之间: {self.relative_bearing}")
        
        # 验证距离非负
        if self.distance < 0:
            raise ValueError(f"距离不能为负数: {self.distance}")
        
        # 验证速度比非负
        if self.speed_ratio < 0:
            raise ValueError(f"速度比不能为负数: {self.speed_ratio}")


@dataclass
class CollisionRiskFactors:
    """碰撞风险因子
    
    用于CRI计算的中间因子
    
    Attributes:
        distance_factor: 距离因子（0-1，距离越近风险越高）
        time_factor: 时间因子（0-1，时间越短风险越高）
        bearing_factor: 方位因子（0-1，考虑相对方位的危险性）
        speed_factor: 速度因子（0-1，速度比越大风险越高）
    """
    distance_factor: float  # 距离因子（0-1）
    time_factor: float      # 时间因子（0-1）
    bearing_factor: float   # 方位因子（0-1）
    speed_factor: float     # 速度因子（0-1）
    
    def __post_init__(self):
        """数据验证"""
        # 验证所有因子都在0-1范围内
        for name, value in [
            ("distance_factor", self.distance_factor),
            ("time_factor", self.time_factor),
            ("bearing_factor", self.bearing_factor),
            ("speed_factor", self.speed_factor)
        ]:
            if not (0 <= value <= 1):
                raise ValueError(f"{name}必须在0到1之间: {value}")


# ============================================================================
# 辅助函数
# ============================================================================

def degrees_to_nautical_miles(degrees: float) -> float:
    """将纬度/经度度数转换为海里（近似）
    
    Args:
        degrees: 度数
        
    Returns:
        海里数
    """
    return degrees * DEGREES_TO_NM


def nautical_miles_to_degrees(nautical_miles: float) -> float:
    """将海里转换为纬度/经度度数（近似）
    
    Args:
        nautical_miles: 海里数
        
    Returns:
        度数
    """
    return nautical_miles / DEGREES_TO_NM


def normalize_angle(angle: float) -> float:
    """将角度归一化到[-180, 180]范围
    
    Args:
        angle: 角度（度）
        
    Returns:
        归一化后的角度
    """
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


def determine_risk_level(cri: float) -> RiskLevel:
    """根据CRI值确定风险等级
    
    Args:
        cri: 碰撞风险指数（0-1）
        
    Returns:
        风险等级
        
    Requirements: 4.3, 4.4
    """
    if cri >= RISK_THRESHOLD_DANGER:
        return RiskLevel.DANGER
    elif cri >= RISK_THRESHOLD_WARNING:
        return RiskLevel.WARNING
    else:
        return RiskLevel.SAFE


# ============================================================================
# DCPA/TCPA 计算函数
# ============================================================================

def calculate_dcpa_tcpa(own_ship, target_ship) -> Tuple[float, float]:
    """计算DCPA (最近会遇距离) 和 TCPA (到达最近点时间)
    
    使用相对运动矢量法计算两船的最近会遇距离和时间。
    
    算法说明：
    1. 计算相对位置矢量 (dx, dy)
    2. 计算相对速度矢量 (dvx, dvy)
    3. 使用投影法计算TCPA：tcpa = -(dr · dv) / |dv|²
    4. 计算TCPA时刻的距离作为DCPA
    
    边界情况处理：
    - 相对速度为零（两船平行同速）：TCPA = inf, DCPA = 当前距离
    - TCPA为负（两船正在远离）：TCPA = 0, DCPA = 当前距离
    - 船舶位置重合：DCPA = 0, TCPA = 0
    
    Args:
        own_ship: 本船状态（ShipState对象）
        target_ship: 目标船状态（ShipState对象）
        
    Returns:
        (dcpa, tcpa): 元组
            - dcpa: 最近会遇距离（海里）
            - tcpa: 到达最近会遇点时间（分钟）
            
    Requirements: 4.1
    
    Examples:
        >>> own = ShipState(mmsi=123, latitude=0.0, longitude=0.0, heading=90, sog=10)
        >>> target = ShipState(mmsi=456, latitude=0.0, longitude=0.1, heading=270, sog=10)
        >>> dcpa, tcpa = calculate_dcpa_tcpa(own, target)
    """
    # 1. 计算相对位置矢量（单位：度）
    # dx: 经度差（东向为正）
    # dy: 纬度差（北向为正）
    dx = target_ship.longitude - own_ship.longitude
    dy = target_ship.latitude - own_ship.latitude
    
    # 转换为海里（1度纬度 ≈ 60海里）
    # 经度需要考虑纬度修正：cos(latitude)
    avg_latitude = (own_ship.latitude + target_ship.latitude) / 2.0
    dx_nm = dx * DEGREES_TO_NM * math.cos(math.radians(avg_latitude))
    dy_nm = dy * DEGREES_TO_NM
    
    # 2. 计算相对速度矢量（单位：节）
    # 使用 vx, vy 属性（已在 ShipState 中定义）
    # vx: 东向速度分量
    # vy: 北向速度分量
    dvx = target_ship.vx - own_ship.vx
    dvy = target_ship.vy - own_ship.vy
    
    # 3. 计算相对速度的平方
    dv_squared = dvx * dvx + dvy * dvy
    
    # 边界情况1：相对速度为零（两船平行同速或都静止）
    if dv_squared < 1e-6:  # 使用小阈值避免浮点误差
        # 两船保持恒定距离，不会相遇
        current_distance = math.sqrt(dx_nm * dx_nm + dy_nm * dy_nm)
        return current_distance, float('inf')
    
    # 4. 计算TCPA（使用投影法）
    # tcpa = -(dr · dv) / |dv|²
    # 其中 dr 是相对位置矢量，dv 是相对速度矢量
    dot_product = dx_nm * dvx + dy_nm * dvy
    tcpa_hours = -dot_product / dv_squared
    
    # 边界情况2：TCPA为负（两船正在远离）
    if tcpa_hours < 0:
        # 最近点已经过去，当前距离就是最近距离
        current_distance = math.sqrt(dx_nm * dx_nm + dy_nm * dy_nm)
        return current_distance, 0.0
    
    # 5. 计算DCPA（TCPA时刻的距离）
    # 位置 = 当前位置 + 速度 × 时间
    future_dx = dx_nm + dvx * tcpa_hours
    future_dy = dy_nm + dvy * tcpa_hours
    dcpa = math.sqrt(future_dx * future_dx + future_dy * future_dy)
    
    # 6. 转换TCPA单位：小时 -> 分钟
    tcpa_minutes = tcpa_hours * 60.0
    
    return dcpa, tcpa_minutes
