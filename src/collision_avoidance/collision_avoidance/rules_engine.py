"""
COLREGS规则引擎模块 (Rules Engine Module)

该模块负责实现国际海上避碰规则（COLREGS）的判定和应用，包括：
- 相遇类型识别（对遇、交叉、追越）
- COLREGS规则应用
- 避让动作决策

Requirements: 3.1-3.6
"""

from dataclasses import dataclass
from enum import Enum
from typing import Optional, List
import math


# ============================================================================
# 常量定义
# ============================================================================

# 相遇类型判定的角度阈值（度）
HEADING_DIFF_HEAD_ON_MIN = 170.0    # 对遇：航向差最小值
HEADING_DIFF_HEAD_ON_MAX = 190.0    # 对遇：航向差最大值
RELATIVE_BEARING_HEAD_ON_MIN = -10.0  # 对遇：相对方位最小值
RELATIVE_BEARING_HEAD_ON_MAX = 10.0   # 对遇：相对方位最大值

RELATIVE_BEARING_CROSSING_MIN = 5.0     # 交叉：相对方位最小值
RELATIVE_BEARING_CROSSING_MAX = 112.5   # 交叉：相对方位最大值

RELATIVE_BEARING_OVERTAKING = 112.5     # 追越：后方扇形区域边界（22.5度扇形）

# COLREGS规则要求的避让角度（度）
TURN_ANGLE_HEAD_ON = 15.0          # 对遇：向右转向角度
TURN_ANGLE_CROSSING_GIVE_WAY = 30.0  # 交叉让路船：大角度避让
TURN_ANGLE_OVERTAKING = 20.0       # 追越：避让角度

# 速度调整因子
SPEED_REDUCTION_FACTOR = 0.7       # 减速避让：速度降至70%


# ============================================================================
# 枚举类型定义
# ============================================================================

class EncounterType(Enum):
    """相遇类型枚举
    
    根据COLREGS规则定义的船舶相遇类型：
    - HEAD_ON: 对遇（Rule 14）- 两船相向航行
    - CROSSING: 交叉相遇（Rule 15）- 两船交叉航行
    - OVERTAKING: 追越（Rule 13）- 一船从后方追赶另一船
    - NONE: 无相遇风险
    
    Requirements: 3.1-3.4
    """
    HEAD_ON = "head_on"          # 对遇 (Rule 14)
    CROSSING = "crossing"        # 交叉相遇 (Rule 15)
    OVERTAKING = "overtaking"    # 追越 (Rule 13)
    NONE = "none"                # 无相遇风险


class VesselRole(Enum):
    """船舶角色枚举
    
    在相遇场景中的船舶角色：
    - GIVE_WAY: 让路船 - 有义务避让的船舶
    - STAND_ON: 直航船 - 有权保持航向航速的船舶
    - BOTH_GIVE_WAY: 双方让路 - 对遇时双方都需避让
    - UNDEFINED: 未定义 - 无明确角色
    
    Requirements: 3.2, 3.3
    """
    GIVE_WAY = "give_way"        # 让路船
    STAND_ON = "stand_on"        # 直航船
    BOTH_GIVE_WAY = "both_give_way"  # 双方让路（对遇）
    UNDEFINED = "undefined"      # 未定义


class TurnDirection(Enum):
    """转向方向枚举
    
    船舶转向方向：
    - STARBOARD: 右转（向右舷转向）
    - PORT: 左转（向左舷转向）
    - NONE: 不转向
    """
    STARBOARD = "starboard"      # 右转
    PORT = "port"                # 左转
    NONE = "none"                # 不转向


class ActionType(Enum):
    """避让动作类型枚举
    
    避让动作的类型：
    - COURSE_CHANGE: 改变航向
    - SPEED_CHANGE: 改变速度
    - COMBINED: 组合动作（航向+速度）
    - MAINTAIN: 保持航向航速
    - NO_ACTION: 无需动作
    """
    COURSE_CHANGE = "course_change"  # 改变航向
    SPEED_CHANGE = "speed_change"    # 改变速度
    COMBINED = "combined"            # 组合动作
    MAINTAIN = "maintain"            # 保持航向航速
    NO_ACTION = "no_action"          # 无需动作


# ============================================================================
# 数据类定义
# ============================================================================

@dataclass
class AvoidanceAction:
    """避让动作数据模型
    
    描述船舶应采取的避让动作，包括转向、速度调整等。
    
    Attributes:
        action_type: 动作类型
        turn_direction: 转向方向（可选）
        turn_angle: 转向角度（度，可选）
        speed_factor: 速度调整因子（可选，1.0表示不变）
        maintain_course: 是否保持航向
        no_action: 是否无需动作
        reason: 动作理由（用于日志和调试）
    
    Requirements: 3.1-3.6, 5.5
    """
    action_type: ActionType
    turn_direction: Optional[TurnDirection] = None
    turn_angle: Optional[float] = None
    speed_factor: Optional[float] = None
    maintain_course: bool = False
    no_action: bool = False
    reason: str = ""
    
    def __post_init__(self):
        """数据验证"""
        # 验证转向角度范围
        if self.turn_angle is not None:
            if not (0 <= self.turn_angle <= 180):
                raise ValueError(f"转向角度必须在0到180度之间: {self.turn_angle}")
        
        # 验证速度因子范围
        if self.speed_factor is not None:
            if not (0 < self.speed_factor <= 2.0):
                raise ValueError(f"速度因子必须在0到2.0之间: {self.speed_factor}")
        
        # 验证动作一致性
        if self.action_type == ActionType.COURSE_CHANGE:
            if self.turn_direction is None or self.turn_angle is None:
                raise ValueError("航向改变动作必须指定转向方向和角度")
        
        if self.action_type == ActionType.SPEED_CHANGE:
            if self.speed_factor is None:
                raise ValueError("速度改变动作必须指定速度因子")
        
        if self.action_type == ActionType.COMBINED:
            if (self.turn_direction is None or self.turn_angle is None or 
                self.speed_factor is None):
                raise ValueError("组合动作必须指定转向方向、角度和速度因子")
        
        if self.action_type == ActionType.MAINTAIN:
            if not self.maintain_course:
                raise ValueError("保持动作必须设置maintain_course=True")
        
        if self.action_type == ActionType.NO_ACTION:
            if not self.no_action:
                raise ValueError("无动作必须设置no_action=True")


@dataclass
class EncounterSituation:
    """相遇态势数据模型
    
    描述两船相遇的完整态势信息，用于规则判定。
    
    Attributes:
        encounter_type: 相遇类型
        own_ship_role: 本船角色
        target_ship_role: 目标船角色
        relative_bearing: 相对方位（度，-180到180）
        heading_difference: 航向差（度，0到180）
        distance: 当前距离（海里）
        is_overtaking: 是否为追越态势
        is_crossing_from_starboard: 目标船是否从右舷交叉
    
    Requirements: 3.1-3.4
    """
    encounter_type: EncounterType
    own_ship_role: VesselRole
    target_ship_role: VesselRole
    relative_bearing: float
    heading_difference: float
    distance: float
    is_overtaking: bool = False
    is_crossing_from_starboard: bool = False
    
    def __post_init__(self):
        """数据验证"""
        # 验证相对方位范围
        if not (-180 <= self.relative_bearing <= 180):
            raise ValueError(f"相对方位必须在-180到180度之间: {self.relative_bearing}")
        
        # 验证航向差范围
        if not (0 <= self.heading_difference <= 180):
            raise ValueError(f"航向差必须在0到180度之间: {self.heading_difference}")
        
        # 验证距离非负
        if self.distance < 0:
            raise ValueError(f"距离不能为负数: {self.distance}")


@dataclass
class COLREGSRule:
    """COLREGS规则数据模型
    
    描述适用的COLREGS规则及其要求。
    
    Attributes:
        rule_number: 规则编号（如"Rule 14"）
        rule_name: 规则名称（如"对遇局面"）
        description: 规则描述
        required_action: 要求的避让动作
        priority: 规则优先级（数字越小优先级越高）
    """
    rule_number: str
    rule_name: str
    description: str
    required_action: str
    priority: int = 0
    
    def __post_init__(self):
        """数据验证"""
        if not self.rule_number or not self.rule_number.strip():
            raise ValueError("规则编号不能为空")
        
        if not self.rule_name or not self.rule_name.strip():
            raise ValueError("规则名称不能为空")
        
        if self.priority < 0:
            raise ValueError(f"规则优先级不能为负数: {self.priority}")


# ============================================================================
# COLREGS规则定义
# ============================================================================

# 预定义的COLREGS规则
COLREGS_RULES = {
    EncounterType.HEAD_ON: COLREGSRule(
        rule_number="Rule 14",
        rule_name="对遇局面",
        description="当两艘机动船对遇或接近对遇以致构成碰撞危险时，各船应向右转向，从对方的左舷驶过。",
        required_action="双方向右转向",
        priority=1
    ),
    EncounterType.CROSSING: COLREGSRule(
        rule_number="Rule 15",
        rule_name="交叉相遇局面",
        description="当两艘机动船交叉相遇以致构成碰撞危险时，有他船在本船右舷的船舶应给他船让路。",
        required_action="让路船采取明显避让行动",
        priority=2
    ),
    EncounterType.OVERTAKING: COLREGSRule(
        rule_number="Rule 13",
        rule_name="追越局面",
        description="任何船舶追越他船时，应给被追越船让路。",
        required_action="追越船避让被追越船",
        priority=3
    ),
}


# ============================================================================
# 辅助函数
# ============================================================================

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


def calculate_heading_difference(heading1: float, heading2: float) -> float:
    """计算两个航向之间的最小夹角
    
    Args:
        heading1: 航向1（度，0-360）
        heading2: 航向2（度，0-360）
        
    Returns:
        航向差（度，0-180）
    """
    diff = abs(heading1 - heading2)
    if diff > 180:
        diff = 360 - diff
    return diff


def calculate_relative_bearing(own_ship, target_ship) -> float:
    """计算目标船相对于本船的方位角
    
    相对方位定义：
    - 0度：目标船在本船正前方
    - 90度：目标船在本船右舷
    - -90度：目标船在本船左舷
    - 180度/-180度：目标船在本船正后方
    
    Args:
        own_ship: 本船状态（ShipState对象）
        target_ship: 目标船状态（ShipState对象）
        
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


def is_crossing_from_starboard(relative_bearing: float) -> bool:
    """判断目标船是否从右舷交叉
    
    Args:
        relative_bearing: 相对方位（度，-180到180）
        
    Returns:
        True表示从右舷交叉，False表示从左舷交叉
    """
    # 右舷：相对方位在0到180度之间
    return 0 < relative_bearing < 180


# ============================================================================
# 核心功能函数（待实现）
# ============================================================================

def determine_encounter_type(own_ship, target_ship) -> EncounterType:
    """判定相遇类型
    
    根据相对方位和航向判断相遇类型。
    
    判定规则：
    1. 追越判定（优先级最高）：目标船在本船后方22.5度扇形区域内
    2. 对遇判定：相向航行，航向差接近180度，相对方位接近0度
    3. 交叉判定：其他情况
    
    Args:
        own_ship: 本船状态（ShipState对象）
        target_ship: 目标船状态（ShipState对象）
        
    Returns:
        相遇类型
        
    Requirements: 3.1-3.4
    
    Note:
        此函数将在任务7.2中实现
    """
    # TODO: 在任务7.2中实现
    raise NotImplementedError("此函数将在任务7.2中实现")


def determine_vessel_roles(encounter_type: EncounterType, 
                          own_ship, 
                          target_ship) -> tuple[VesselRole, VesselRole]:
    """确定船舶角色（让路船/直航船）
    
    根据相遇类型和相对位置确定本船和目标船的角色。
    
    Args:
        encounter_type: 相遇类型
        own_ship: 本船状态
        target_ship: 目标船状态
        
    Returns:
        (本船角色, 目标船角色)
        
    Requirements: 3.2, 3.3
    
    Note:
        此函数将在任务7.2中实现
    """
    # TODO: 在任务7.2中实现
    raise NotImplementedError("此函数将在任务7.2中实现")


def apply_colregs_rule(encounter_type: EncounterType, 
                       own_ship, 
                       target_ship) -> AvoidanceAction:
    """应用COLREGS规则，返回避让动作
    
    根据相遇类型应用相应的COLREGS规则，生成避让动作。
    
    Args:
        encounter_type: 相遇类型
        own_ship: 本船状态
        target_ship: 目标船状态
        
    Returns:
        避让动作
        
    Requirements: 3.1-3.6
    
    Note:
        此函数将在任务7.4中实现
    """
    # TODO: 在任务7.4中实现
    raise NotImplementedError("此函数将在任务7.4中实现")


def analyze_encounter_situation(own_ship, target_ship) -> EncounterSituation:
    """分析相遇态势
    
    综合分析两船的相遇态势，返回完整的态势信息。
    
    Args:
        own_ship: 本船状态
        target_ship: 目标船状态
        
    Returns:
        相遇态势信息
        
    Note:
        此函数将在任务7.2中实现
    """
    # TODO: 在任务7.2中实现
    raise NotImplementedError("此函数将在任务7.2中实现")


def get_applicable_rule(encounter_type: EncounterType) -> COLREGSRule:
    """获取适用的COLREGS规则
    
    Args:
        encounter_type: 相遇类型
        
    Returns:
        适用的COLREGS规则
        
    Raises:
        ValueError: 如果相遇类型为NONE
    """
    if encounter_type == EncounterType.NONE:
        raise ValueError("无相遇风险时不适用COLREGS规则")
    
    return COLREGS_RULES.get(encounter_type)
