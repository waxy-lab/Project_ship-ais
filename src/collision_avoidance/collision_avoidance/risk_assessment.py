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

def calculate_bearing_risk(bearing: float) -> float:
    """计算方位因子（考虑相对方位的危险性）
    
    不同相对方位的危险程度不同：
    - 正前方（0度附近）：最危险，因子接近1
    - 正侧方（90度附近）：中等危险，因子约0.5
    - 正后方（180度附近）：较低危险，因子接近0
    
    使用余弦函数建模：bearing_factor = (1 - cos(bearing)) / 2
    这样：
    - bearing = 0°   -> factor = 0.0 (正前方，最危险)
    - bearing = 90°  -> factor = 0.5 (正侧方，中等危险)
    - bearing = 180° -> factor = 1.0 (正后方，较低危险)
    
    然后反转：1 - factor，使得正前方风险最高
    
    Args:
        bearing: 相对方位（度，-180到180）
        
    Returns:
        方位因子（0-1，值越大风险越高）
    """
    # 取绝对值，因为左右对称
    abs_bearing = abs(bearing)
    
    # 使用余弦函数计算
    # cos(0°) = 1, cos(90°) = 0, cos(180°) = -1
    cos_bearing = math.cos(math.radians(abs_bearing))
    
    # 归一化到0-1，并反转（使正前方风险最高）
    # bearing = 0°   -> cos = 1  -> factor = 1.0 (最危险)
    # bearing = 90°  -> cos = 0  -> factor = 0.5 (中等危险)
    # bearing = 180° -> cos = -1 -> factor = 0.0 (较低危险)
    bearing_factor = (1.0 + cos_bearing) / 2.0
    
    return bearing_factor


def calculate_relative_bearing(own_ship, target_ship) -> float:
    """计算目标船相对于本船的方位角
    
    相对方位定义：
    - 0度：目标船在本船正前方
    - 90度：目标船在本船右侧
    - -90度：目标船在本船左侧
    - 180度/-180度：目标船在本船正后方
    
    Args:
        own_ship: 本船状态
        target_ship: 目标船状态
        
    Returns:
        相对方位（度，-180到180）
    """
    # 计算目标船相对于本船的地理方位
    dx = target_ship.longitude - own_ship.longitude
    dy = target_ship.latitude - own_ship.latitude
    
    # 考虑纬度修正
    avg_latitude = (own_ship.latitude + target_ship.latitude) / 2.0
    dx_corrected = dx * math.cos(math.radians(avg_latitude))
    
    # 计算地理方位角（北向为0度，顺时针）
    geographic_bearing = math.degrees(math.atan2(dx_corrected, dy))
    
    # 转换为相对方位（相对于本船航向）
    relative_bearing = geographic_bearing - own_ship.heading
    
    # 归一化到[-180, 180]
    relative_bearing = normalize_angle(relative_bearing)
    
    return relative_bearing


def calculate_cri(dcpa: float, tcpa: float, bearing: float, speed_ratio: float) -> float:
    """计算综合碰撞风险指数 (CRI)
    
    CRI是一个0-1之间的综合指标，考虑多个风险因素：
    1. 距离因子：DCPA越小，风险越高
    2. 时间因子：TCPA越小，风险越高
    3. 方位因子：正前方风险最高，正后方风险最低
    4. 速度因子：速度比越大，风险越高
    
    计算公式：
    CRI = 0.4 × distance_factor + 0.3 × time_factor + 
          0.2 × bearing_factor + 0.1 × speed_factor
    
    权重分配：
    - 距离因子（40%）：最重要，直接反映碰撞危险
    - 时间因子（30%）：次重要，反映避让紧迫性
    - 方位因子（20%）：考虑相遇几何关系
    - 速度因子（10%）：考虑相对运动特性
    
    Args:
        dcpa: 最近会遇距离（海里）
        tcpa: 到达最近会遇点时间（分钟）
        bearing: 相对方位（度，-180到180）
        speed_ratio: 速度比（目标船速度/本船速度）
        
    Returns:
        碰撞风险指数（0-1，值越大风险越高）
        
    Requirements: 4.2
    
    Examples:
        >>> # 高风险场景：DCPA=0.3海里，TCPA=3分钟
        >>> cri = calculate_cri(dcpa=0.3, tcpa=3.0, bearing=0.0, speed_ratio=1.2)
        >>> assert cri > 0.7  # 应该触发危险阈值
        
        >>> # 低风险场景：DCPA=5海里，TCPA=30分钟
        >>> cri = calculate_cri(dcpa=5.0, tcpa=30.0, bearing=90.0, speed_ratio=0.8)
        >>> assert cri < 0.3  # 应该是安全状态
    """
    # 1. 距离因子（0-1，距离越近风险越高）
    # 使用反比例函数：1 / (1 + dcpa / SAFE_DISTANCE)
    # dcpa = 0        -> factor = 1.0 (最危险)
    # dcpa = SAFE_DISTANCE -> factor = 0.5 (中等危险)
    # dcpa -> ∞       -> factor -> 0.0 (安全)
    distance_factor = 1.0 / (1.0 + dcpa / SAFE_DISTANCE)
    
    # 2. 时间因子（0-1，时间越短风险越高）
    # 处理特殊情况：TCPA为无穷大或负数
    if tcpa <= 0 or math.isinf(tcpa):
        # 两船正在远离或保持恒定距离，时间因子为0
        time_factor = 0.0
    else:
        # 使用反比例函数：1 / (1 + tcpa / SAFE_TIME)
        # tcpa = 0        -> factor = 1.0 (最危险)
        # tcpa = SAFE_TIME -> factor = 0.5 (中等危险)
        # tcpa -> ∞       -> factor -> 0.0 (安全)
        time_factor = 1.0 / (1.0 + tcpa / SAFE_TIME)
    
    # 3. 方位因子（0-1，考虑相对方位的危险性）
    bearing_factor = calculate_bearing_risk(bearing)
    
    # 4. 速度因子（0-1，速度比越大风险越高）
    # 限制速度比在0-2范围内，然后归一化
    # speed_ratio = 0   -> factor = 0.0 (目标船静止)
    # speed_ratio = 1   -> factor = 0.5 (同速)
    # speed_ratio = 2   -> factor = 1.0 (目标船速度是本船2倍)
    # speed_ratio > 2   -> factor = 1.0 (截断)
    clamped_speed_ratio = min(speed_ratio, 2.0)
    speed_factor = clamped_speed_ratio / 2.0
    
    # 5. 加权综合计算CRI
    # 权重：距离40%，时间30%，方位20%，速度10%
    cri = (0.4 * distance_factor + 
           0.3 * time_factor + 
           0.2 * bearing_factor + 
           0.1 * speed_factor)
    
    # 确保CRI在[0, 1]范围内（理论上应该自动满足）
    cri = max(0.0, min(1.0, cri))
    
    return cri


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


# ============================================================================
# 综合风险评估函数
# ============================================================================

def assess_collision_risk(own_ship, target_ship) -> RiskAssessment:
    """综合评估碰撞风险
    
    该函数整合了DCPA/TCPA计算、CRI计算和风险等级判定，
    返回完整的风险评估结果。
    
    Args:
        own_ship: 本船状态（ShipState对象）
        target_ship: 目标船状态（ShipState对象）
        
    Returns:
        RiskAssessment对象，包含所有风险指标
        
    Requirements: 4.1, 4.2, 4.3, 4.4
    
    Examples:
        >>> own = ShipState(mmsi=123, latitude=0.0, longitude=0.0, heading=90, sog=10)
        >>> target = ShipState(mmsi=456, latitude=0.0, longitude=0.05, heading=270, sog=12)
        >>> risk = assess_collision_risk(own, target)
        >>> print(f"CRI: {risk.cri:.2f}, Level: {risk.risk_level}")
    """
    # 1. 计算DCPA和TCPA
    dcpa, tcpa = calculate_dcpa_tcpa(own_ship, target_ship)
    
    # 2. 计算相对方位
    relative_bearing = calculate_relative_bearing(own_ship, target_ship)
    
    # 3. 计算当前距离
    dx = target_ship.longitude - own_ship.longitude
    dy = target_ship.latitude - own_ship.latitude
    avg_latitude = (own_ship.latitude + target_ship.latitude) / 2.0
    dx_nm = dx * DEGREES_TO_NM * math.cos(math.radians(avg_latitude))
    dy_nm = dy * DEGREES_TO_NM
    current_distance = math.sqrt(dx_nm * dx_nm + dy_nm * dy_nm)
    
    # 4. 计算速度比
    if own_ship.sog < 1e-6:  # 避免除零
        speed_ratio = 0.0
    else:
        speed_ratio = target_ship.sog / own_ship.sog
    
    # 5. 计算CRI
    cri = calculate_cri(dcpa, tcpa, relative_bearing, speed_ratio)
    
    # 6. 确定风险等级
    risk_level = determine_risk_level(cri)
    
    # 7. 构建并返回风险评估结果
    return RiskAssessment(
        dcpa=dcpa,
        tcpa=tcpa,
        cri=cri,
        risk_level=risk_level,
        relative_bearing=relative_bearing,
        distance=current_distance,
        speed_ratio=speed_ratio
    )

