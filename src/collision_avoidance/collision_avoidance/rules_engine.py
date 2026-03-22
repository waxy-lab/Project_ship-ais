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
TURN_ANGLE_OVERTAKING = 40.0       # 追越：避让角度（大角度确保绕开）

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
    RESUME_COURSE = "resume_course"  # 回正航向（追越完成后恢复原航向）


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


def is_overtaking_complete(own_ship, target_ship,
                           safe_lateral_nm: float = 0.3,
                           safe_ahead_nm: float = 0.1) -> bool:
    """
    判断追越是否已完成：本船已超过目标船并保持足够横向间距。

    完成条件（同时满足）：
    1. 本船在目标船前方（沿目标船航向投影，本船超前 safe_ahead_nm 海里）
    2. 本船与目标船横向间距 > safe_lateral_nm 海里（已离开目标船航迹线）

    Args:
        own_ship:        本船状态
        target_ship:     目标船状态
        safe_lateral_nm: 安全横向间距（海里），默认 0.3nm
        safe_ahead_nm:   超前距离（海里），默认 0.1nm

    Returns:
        True 表示追越完成，可以回正航向
    """
    NM = 60.0  # 1度纬度 ≈ 60海里
    cos_lat = math.cos(math.radians(target_ship.latitude))

    # 目标船航向单位向量（NE坐标系，北=y，东=x）
    hdg_rad = math.radians(target_ship.heading)
    along_north = math.cos(hdg_rad)   # 北向分量
    along_east  = math.sin(hdg_rad)   # 东向分量
    # 横向（右舷正方向）
    perp_north = -math.sin(hdg_rad)
    perp_east  =  math.cos(hdg_rad)

    # 本船相对目标船的位移（海里，北为正，东为正）
    dn = (own_ship.latitude  - target_ship.latitude)  * NM
    de = (own_ship.longitude - target_ship.longitude) * NM * cos_lat

    # 沿目标航向投影（正值 = 本船在目标前方）
    along_nm = dn * along_north + de * along_east
    # 横向间距（绝对值）
    lateral_nm = abs(dn * perp_north + de * perp_east)

    return along_nm > safe_ahead_nm and lateral_nm > safe_lateral_nm


def is_avoidance_complete(own_ship, target_ship,
                          safe_pass_nm: float = 0.5) -> bool:
    """
    判断对遇/交叉避让是否已完成：目标船已从本船后方安全通过。

    完成条件：
    - 目标船在本船正后方（沿本船航向投影为负值，即已超过本船）
    - 两船距离 > safe_pass_nm 海里（已形成足够安全间距）

    Args:
        own_ship:      本船状态
        target_ship:   目标船状态
        safe_pass_nm:  安全通过距离（海里），默认 0.5nm

    Returns:
        True 表示避让完成，可以回正航向
    """
    NM = 60.0
    cos_lat = math.cos(math.radians(own_ship.latitude))

    # 本船航向单位向量
    hdg_rad = math.radians(own_ship.heading)
    along_north = math.cos(hdg_rad)
    along_east  = math.sin(hdg_rad)

    # 目标相对本船的位移（海里）
    dn = (target_ship.latitude  - own_ship.latitude)  * NM
    de = (target_ship.longitude - own_ship.longitude) * NM * cos_lat

    # 沿本船航向投影（负值 = 目标已在本船后方）
    along_nm = dn * along_north + de * along_east

    # 两船距离
    dist_nm = math.hypot(dn, de)

    return along_nm < -0.1 and dist_nm > safe_pass_nm


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
    """
    # 计算相对方位和航向差
    relative_bearing = calculate_relative_bearing(own_ship, target_ship)
    heading_diff = calculate_heading_difference(own_ship.heading, target_ship.heading)
    
    # 1. 追越判定（优先级最高）
    # COLREGS Rule 13：追越船定义为「从被追越船正后方22.5度以内的任何方向追及他船」
    # 两种等价几何：
    #   a) 目标在本船后方（目标追本船）：abs(relative_bearing) > 112.5 且目标更快
    #   b) 目标在本船前方（本船追目标）：abs(relative_bearing) < 67.5 且两船同向 且本船更快
    #      — 此时从目标视角看本船在目标后方22.5扇区内，本船是追越船，需主动避让
    
    # 情形 a：目标从后方追来，目标是追越船（本船被追越，直航船）
    if (abs(relative_bearing) > RELATIVE_BEARING_OVERTAKING and
            target_ship.speed > own_ship.speed):
        return EncounterType.OVERTAKING
    
    # 情形 b：本船从后方追目标，本船是追越船（需让路）
    # 条件：目标在本船前方扇区（abs(bearing) < 67.5°）、同向（heading_diff < 30°）、本船更快
    if (abs(relative_bearing) < (180.0 - RELATIVE_BEARING_OVERTAKING) and
            heading_diff < 30.0 and
            own_ship.speed > target_ship.speed + 0.5):
        return EncounterType.OVERTAKING
    
    # 2. 对遇判定
    # 对遇：航向差在170-190度之间，且相对方位在-10到10度之间
    if (HEADING_DIFF_HEAD_ON_MIN <= heading_diff <= HEADING_DIFF_HEAD_ON_MAX and
        RELATIVE_BEARING_HEAD_ON_MIN <= relative_bearing <= RELATIVE_BEARING_HEAD_ON_MAX):
        return EncounterType.HEAD_ON
    
    # 3. 交叉判定
    # 交叉：相对方位在5-112.5度或-112.5到-5度之间
    if (RELATIVE_BEARING_CROSSING_MIN <= abs(relative_bearing) <= RELATIVE_BEARING_CROSSING_MAX):
        return EncounterType.CROSSING
    
    # 4. 无相遇风险
    return EncounterType.NONE


def apply_restricted_water_rule(own_ship, target_ship) -> 'AvoidanceAction':
    """
    Rule 9：狭水道/航道规则

    在受限水域中，船舶应靠近本身右舷的外侧边界行驶，
    对遇时不应采用大角度右转（航道太窄），而是减速并保持靠右行驶。
    追越在狭水道中须谨慎，仅在安全时进行。

    Args:
        own_ship:    本船状态
        target_ship: 目标船状态

    Returns:
        AvoidanceAction
    """
    enc = determine_encounter_type(own_ship, target_ship)
    relative_bearing = calculate_relative_bearing(own_ship, target_ship)

    # 对遇：狭水道中靠右各行其道，不大角度转向，减速让对方通过
    if enc == EncounterType.HEAD_ON:
        return AvoidanceAction(
            action_type=ActionType.SPEED_CHANGE,
            speed_factor=0.5,
            reason="Rule 9: 狭水道对遇，减速靠右行驶，等待对方通过"
        )

    # 交叉：狭水道中不应横越，保持航向减速
    if enc == EncounterType.CROSSING:
        own_role, _ = determine_vessel_roles(enc, own_ship, target_ship)
        if own_role == VesselRole.GIVE_WAY:
            return AvoidanceAction(
                action_type=ActionType.SPEED_CHANGE,
                speed_factor=0.6,
                reason="Rule 9: 狭水道交叉，减速让路"
            )
        return AvoidanceAction(
            action_type=ActionType.MAINTAIN,
            maintain_course=True,
            reason="Rule 9: 狭水道交叉，本船直航船，保持航向"
        )

    # 追越：狭水道追越须谨慎，同样适用 Rule 13
    if enc == EncounterType.OVERTAKING:
        own_role, _ = determine_vessel_roles(enc, own_ship, target_ship)
        if own_role == VesselRole.GIVE_WAY:
            if is_overtaking_complete(own_ship, target_ship):
                return AvoidanceAction(
                    action_type=ActionType.RESUME_COURSE,
                    reason="Rule 13/9: 狭水道追越完成，回正航向"
                )
            return AvoidanceAction(
                action_type=ActionType.SPEED_CHANGE,
                speed_factor=0.7,
                reason="Rule 9/13: 狭水道追越，减速等待安全超越机会"
            )

    # 无风险或已通过
    if is_avoidance_complete(own_ship, target_ship):
        return AvoidanceAction(
            action_type=ActionType.RESUME_COURSE,
            reason="Rule 9: 狭水道避让完成，恢复正常航行"
        )
    return AvoidanceAction(
        action_type=ActionType.MAINTAIN,
        maintain_course=True,
        reason="Rule 9: 狭水道，保持航向航速"
    )


def determine_vessel_roles(encounter_type: EncounterType, 
                          own_ship, 
                          target_ship) -> tuple[VesselRole, VesselRole]:
    """确定船舶角色（让路船/直航船）
    
    根据相遇类型和相对位置确定本船和目标船的角色。
    
    规则：
    - 对遇：双方都是让路船
    - 交叉：右舷有他船的船舶为让路船
    - 追越：追越船为让路船
    
    Args:
        encounter_type: 相遇类型
        own_ship: 本船状态
        target_ship: 目标船状态
        
    Returns:
        (本船角色, 目标船角色)
        
    Requirements: 3.2, 3.3
    """
    if encounter_type == EncounterType.NONE:
        return (VesselRole.UNDEFINED, VesselRole.UNDEFINED)
    
    # 对遇：双方都需要让路
    if encounter_type == EncounterType.HEAD_ON:
        return (VesselRole.BOTH_GIVE_WAY, VesselRole.BOTH_GIVE_WAY)
    
    # 交叉：判断目标船是否在右舷
    if encounter_type == EncounterType.CROSSING:
        relative_bearing = calculate_relative_bearing(own_ship, target_ship)
        if is_crossing_from_starboard(relative_bearing):
            # 目标船在右舷，本船为让路船
            return (VesselRole.GIVE_WAY, VesselRole.STAND_ON)
        else:
            # 目标船在左舷，本船为直航船
            return (VesselRole.STAND_ON, VesselRole.GIVE_WAY)
    
    # 追越：追越船为让路船
    if encounter_type == EncounterType.OVERTAKING:
        # 判断谁是追越船（速度更快的船）
        if own_ship.speed > target_ship.speed:
            # 本船追越目标船，本船为让路船
            return (VesselRole.GIVE_WAY, VesselRole.STAND_ON)
        else:
            # 目标船追越本船，本船为直航船
            return (VesselRole.STAND_ON, VesselRole.GIVE_WAY)
    
    return (VesselRole.UNDEFINED, VesselRole.UNDEFINED)


def apply_colregs_rule(encounter_type: EncounterType,
                       own_ship,
                       target_ship,
                       environment=None) -> AvoidanceAction:
    """应用COLREGS规则，返回避让动作

    根据相遇类型应用相应的COLREGS规则，生成避让动作。
    environment 为 EnvironmentConfig 对象，传入时根据天气/能见度调整策略：
    - 恶劣天气：增大避让角度（风流导致实际轨迹偏移，需更大余量）
    - 能见度不良：Rule 19，雾中减速，避让角度更大更保守

    规则应用逻辑：
    1. 对遇（Rule 14）：双方向右转向
    2. 交叉相遇（Rule 15/17）：
       - 让路船：采取明显避让行动（大角度右转）
       - 直航船：保持航向和航速
    3. 追越（Rule 13）：追越船避让被追越船
    4. 无相遇风险：无需动作
    
    Args:
        encounter_type: 相遇类型
        own_ship: 本船状态
        target_ship: 目标船状态
        
    Returns:
        避让动作
        
    Requirements: 3.1-3.6
    """
    # ---------- 环境因素：根据天气/能见度调整避让角度 ----------
    _env_turn_bonus = 0.0    # 额外转向角度（度）
    _env_speed_factor = 1.0  # 速度修正系数
    _fog_mode = False        # 能见度不良模式（Rule 19）

    if environment is not None:
        from scenario_generator.models import WeatherCondition, Visibility
        wc = getattr(environment, 'weather_condition', None)
        vis = getattr(environment, 'visibility', None)
        wind_spd = getattr(environment, 'wind_speed', 0.0)     # m/s
        cur_spd  = getattr(environment, 'current_speed', 0.0)  # m/s

        # 恶劣天气：风速 > 10m/s 或天气 rough，增大避让角度
        if (wc == WeatherCondition.ROUGH or wind_spd > 10.0):
            _env_turn_bonus += 10.0   # 额外10°，补偿风流漂移
            _env_speed_factor = 0.85  # 略微减速
        elif (wc == WeatherCondition.MODERATE or wind_spd > 5.0):
            _env_turn_bonus += 5.0
            _env_speed_factor = 0.92

        # 能见度不良：Rule 19，雾中保守操作
        if vis == Visibility.POOR:
            _env_turn_bonus += 15.0   # 更大避让角度
            _env_speed_factor = min(_env_speed_factor, 0.7)  # 减速至安全速度
            _fog_mode = True
        elif vis == Visibility.MODERATE:
            _env_turn_bonus += 8.0
            _env_speed_factor = min(_env_speed_factor, 0.85)

    # ---------- COLREGS 规则判定 ----------
    # 无相遇风险：检查是否刚完成避让，若是则发出回正指令
    if encounter_type == EncounterType.NONE:
        # 追越完成回正
        if is_overtaking_complete(own_ship, target_ship):
            return AvoidanceAction(
                action_type=ActionType.RESUME_COURSE,
                reason="Rule 13: 追越完成，本船已超过目标船并保持安全间距，回正航向"
            )
        # 对遇/交叉避让完成：目标船已通过且横向间距足够，回正
        if is_avoidance_complete(own_ship, target_ship):
            return AvoidanceAction(
                action_type=ActionType.RESUME_COURSE,
                reason="避让完成，目标船已安全通过，回正至原始航向"
            )
        return AvoidanceAction(
            action_type=ActionType.NO_ACTION,
            no_action=True,
            reason="无相遇风险，无需避让动作"
        )
    
    # 对遇（Rule 14）：双方向右转向
    if encounter_type == EncounterType.HEAD_ON:
        angle = TURN_ANGLE_HEAD_ON + _env_turn_bonus
        reason = "Rule 14: 对遇局面，向右转向"
        if _fog_mode:
            reason = "Rule 19: 能见度不良对遇，大幅右转并减速"
        elif _env_turn_bonus > 0:
            reason = f"Rule 14: 对遇局面，恶劣天气增大转向角至{angle:.0f}°"
        action = AvoidanceAction(
            action_type=ActionType.COURSE_CHANGE,
            turn_direction=TurnDirection.STARBOARD,
            turn_angle=angle,
            reason=reason
        )
        # 能见度不良/恶劣天气同时减速
        if _env_speed_factor < 1.0:
            return AvoidanceAction(
                action_type=ActionType.COURSE_CHANGE,
                turn_direction=TurnDirection.STARBOARD,
                turn_angle=angle,
                speed_factor=_env_speed_factor,
                reason=reason
            )
        return action
    
    # 交叉相遇（Rule 15/17）：判断让路船和直航船
    if encounter_type == EncounterType.CROSSING:
        # 确定船舶角色
        own_role, target_role = determine_vessel_roles(encounter_type, own_ship, target_ship)
        
        if own_role == VesselRole.GIVE_WAY:
            # 让路船：采取明显避让行动（大角度右转）
            angle = TURN_ANGLE_CROSSING_GIVE_WAY + _env_turn_bonus
            reason = "Rule 15: 交叉相遇，本船为让路船，采取明显避让行动"
            if _fog_mode:
                reason = f"Rule 19/15: 能见度不良交叉，大幅右转{angle:.0f}°并减速"
            elif _env_turn_bonus > 0:
                reason = f"Rule 15: 交叉相遇，恶劣天气增大转向角至{angle:.0f}°"
            return AvoidanceAction(
                action_type=ActionType.COURSE_CHANGE,
                turn_direction=TurnDirection.STARBOARD,
                turn_angle=angle,
                speed_factor=_env_speed_factor if _env_speed_factor < 1.0 else None,
                reason=reason
            )
        elif own_role == VesselRole.STAND_ON:
            # 直航船：保持航向和航速
            return AvoidanceAction(
                action_type=ActionType.MAINTAIN,
                maintain_course=True,
                reason="Rule 17: 交叉相遇，本船为直航船，保持航向和航速"
            )
        else:
            # 角色未定义，采取保守策略
            return AvoidanceAction(
                action_type=ActionType.NO_ACTION,
                no_action=True,
                reason="交叉相遇，角色未定义，无法确定避让动作"
            )
    
    # 追越（Rule 13）：追越船避让被追越船
    if encounter_type == EncounterType.OVERTAKING:
        # 确定船舶角色
        own_role, target_role = determine_vessel_roles(encounter_type, own_ship, target_ship)
        
        if own_role == VesselRole.GIVE_WAY:
            # 本船是追越船
            # 先检查是否已完成超越，若是则回正航向（类似汽车超车后并回原车道）
            if is_overtaking_complete(own_ship, target_ship):
                return AvoidanceAction(
                    action_type=ActionType.RESUME_COURSE,
                    reason="Rule 13: 追越完成，本船已超过目标船并保持安全间距，回正航向"
                )
            
            # 尚未完成超越，需要避让转向
            # 计算相对方位，决定从哪一侧避让
            relative_bearing = calculate_relative_bearing(own_ship, target_ship)
            
            # 目标在前方：向右舷绕行（标准追越避让，从目标右侧超越）
            # 目标在右方：向左避让；目标在左方：向右避让
            if abs(relative_bearing) < 67.5:  # 目标在前方，标准追越
                turn_dir = TurnDirection.STARBOARD
            elif relative_bearing < 0:
                turn_dir = TurnDirection.PORT
            else:
                turn_dir = TurnDirection.STARBOARD
            
            return AvoidanceAction(
                action_type=ActionType.COURSE_CHANGE,
                turn_direction=turn_dir,
                turn_angle=TURN_ANGLE_OVERTAKING,
                reason="Rule 13: 追越局面，本船为追越船，避让被追越船"
            )
        elif own_role == VesselRole.STAND_ON:
            # 本船被追越，保持航向和航速
            return AvoidanceAction(
                action_type=ActionType.MAINTAIN,
                maintain_course=True,
                reason="Rule 13: 追越局面，本船为被追越船，保持航向和航速"
            )
        else:
            # 角色未定义，采取保守策略
            return AvoidanceAction(
                action_type=ActionType.NO_ACTION,
                no_action=True,
                reason="追越局面，角色未定义，无法确定避让动作"
            )
    
    # 未知相遇类型（不应该到达这里）
    return AvoidanceAction(
        action_type=ActionType.NO_ACTION,
        no_action=True,
        reason=f"未知相遇类型: {encounter_type}"
    )


def analyze_encounter_situation(own_ship, target_ship) -> EncounterSituation:
    """分析相遇态势
    
    综合分析两船的相遇态势，返回完整的态势信息。
    
    Args:
        own_ship: 本船状态
        target_ship: 目标船状态
        
    Returns:
        相遇态势信息
    """
    # 计算相对方位和航向差
    relative_bearing = calculate_relative_bearing(own_ship, target_ship)
    heading_diff = calculate_heading_difference(own_ship.heading, target_ship.heading)
    
    # 计算距离（简化的欧几里得距离，转换为海里）
    lat_diff = target_ship.latitude - own_ship.latitude
    lon_diff = target_ship.longitude - own_ship.longitude
    # 考虑纬度修正
    avg_latitude = (own_ship.latitude + target_ship.latitude) / 2.0
    lon_diff_corrected = lon_diff * math.cos(math.radians(avg_latitude))
    # 1度约等于60海里
    distance = math.sqrt(lat_diff**2 + lon_diff_corrected**2) * 60.0
    
    # 判定相遇类型
    encounter_type = determine_encounter_type(own_ship, target_ship)
    
    # 确定船舶角色
    own_role, target_role = determine_vessel_roles(encounter_type, own_ship, target_ship)
    
    # 判断是否为追越态势
    is_overtaking = (encounter_type == EncounterType.OVERTAKING)
    
    # 判断是否从右舷交叉
    is_crossing_from_starboard_flag = (
        encounter_type == EncounterType.CROSSING and 
        is_crossing_from_starboard(relative_bearing)
    )
    
    return EncounterSituation(
        encounter_type=encounter_type,
        own_ship_role=own_role,
        target_ship_role=target_role,
        relative_bearing=relative_bearing,
        heading_difference=heading_diff,
        distance=distance,
        is_overtaking=is_overtaking,
        is_crossing_from_starboard=is_crossing_from_starboard_flag
    )


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
