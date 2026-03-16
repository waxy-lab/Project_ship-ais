"""
路径规划模块 (Path Planning Module)

该模块负责生成和评估避让路径，包括：
- 候选路径生成
- 路径安全性评估
- 路径效率评估
- 路径COLREGS合规性评估
- 最优路径选择

Requirements: 5.1-5.6
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional, Tuple
import math

from .risk_assessment import calculate_dcpa_tcpa, SAFE_DISTANCE
from .rules_engine import AvoidanceAction, ActionType, TurnDirection


# ============================================================================
# 常量定义
# ============================================================================

# 路径规划参数
DEFAULT_TURN_ANGLES = [15.0, 30.0, 45.0]   # 默认转向角度选项（度）
DEFAULT_SPEED_FACTORS = [0.7, 0.8, 0.9]    # 默认速度调整因子选项
MIN_SAFE_DISTANCE = 2.0                     # 最小安全距离（海里）
PATH_EVALUATION_HORIZON = 30.0              # 路径评估时间范围（分钟）

# 路径评分权重
WEIGHT_SAFETY = 0.5        # 安全性权重（50%）
WEIGHT_EFFICIENCY = 0.3    # 效率权重（30%）
WEIGHT_COMPLIANCE = 0.2    # 合规性权重（20%）

# 航向改变阈值
MIN_COURSE_CHANGE = 15.0   # 最小有效航向改变（度）
MAX_COURSE_CHANGE = 90.0   # 最大航向改变（度）

# 速度调整阈值
MIN_SPEED_FACTOR = 0.5     # 最小速度因子（50%）
MAX_SPEED_FACTOR = 1.0     # 最大速度因子（100%）

# 路径预测步长（分钟）
PATH_TIME_STEP = 1.0

# 返航触发阈值
DEFAULT_RETURN_DISTANCE = 5.0   # 威胁解除距离（海里）
DEFAULT_RETURN_TIME = 10.0      # 威胁解除时间（分钟）

# 单位转换
DEGREES_TO_NM = 60.0   # 度转海里（近似）


# ============================================================================
# 枚举类型定义
# ============================================================================

class PathType(Enum):
    """路径类型枚举"""
    COURSE_CHANGE = "course_change"
    SPEED_CHANGE = "speed_change"
    COMBINED = "combined"
    MAINTAIN = "maintain"


class PathStatus(Enum):
    """路径状态枚举"""
    SAFE = "safe"
    RISKY = "risky"
    UNSAFE = "unsafe"
    NOT_EVALUATED = "not_evaluated"


# ============================================================================
# 数据类定义
# ============================================================================

@dataclass
class Waypoint:
    """航点数据模型"""
    latitude: float
    longitude: float
    timestamp: float
    heading: float
    speed: float

    def __post_init__(self):
        if not (-90 <= self.latitude <= 90):
            raise ValueError(f"纬度必须在-90到90之间: {self.latitude}")
        if not (-180 <= self.longitude <= 180):
            raise ValueError(f"经度必须在-180到180之间: {self.longitude}")
        if not (0 <= self.heading < 360):
            raise ValueError(f"航向必须在0到360之间: {self.heading}")
        if self.speed < 0:
            raise ValueError(f"速度不能为负数: {self.speed}")
        if self.timestamp < 0:
            raise ValueError(f"时间戳不能为负数: {self.timestamp}")


@dataclass
class Path:
    """路径数据模型

    Requirements: 5.1
    """
    path_id: str
    path_type: PathType
    waypoints: List[Waypoint]
    course_change: Optional[float] = None
    speed_factor: Optional[float] = None
    estimated_duration: float = 0.0
    path_length: float = 0.0
    status: PathStatus = PathStatus.NOT_EVALUATED
    description: str = ""

    def __post_init__(self):
        if not self.path_id or not self.path_id.strip():
            raise ValueError("路径ID不能为空")
        if not self.waypoints or len(self.waypoints) < 2:
            raise ValueError("路径至少需要2个航点")
        if self.course_change is not None:
            if not (-180 <= self.course_change <= 180):
                raise ValueError(f"航向改变量必须在-180到180度之间: {self.course_change}")
        if self.speed_factor is not None:
            if not (0 < self.speed_factor <= 2.0):
                raise ValueError(f"速度因子必须在0到2.0之间: {self.speed_factor}")
        if self.estimated_duration < 0:
            raise ValueError(f"预计持续时间不能为负数: {self.estimated_duration}")
        if self.path_length < 0:
            raise ValueError(f"路径长度不能为负数: {self.path_length}")

    def get_start_waypoint(self) -> Waypoint:
        return self.waypoints[0]

    def get_end_waypoint(self) -> Waypoint:
        return self.waypoints[-1]

    def get_waypoint_at_time(self, timestamp: float) -> Optional[Waypoint]:
        """获取指定时间的航点（线性插值）"""
        if timestamp < self.waypoints[0].timestamp:
            return None
        if timestamp >= self.waypoints[-1].timestamp:
            return self.waypoints[-1]

        for i in range(len(self.waypoints) - 1):
            if self.waypoints[i].timestamp <= timestamp < self.waypoints[i + 1].timestamp:
                t0 = self.waypoints[i].timestamp
                t1 = self.waypoints[i + 1].timestamp
                ratio = (timestamp - t0) / (t1 - t0)
                lat = self.waypoints[i].latitude + ratio * (
                    self.waypoints[i + 1].latitude - self.waypoints[i].latitude)
                lon = self.waypoints[i].longitude + ratio * (
                    self.waypoints[i + 1].longitude - self.waypoints[i].longitude)
                speed = self.waypoints[i].speed + ratio * (
                    self.waypoints[i + 1].speed - self.waypoints[i].speed)
                return Waypoint(
                    latitude=lat,
                    longitude=lon,
                    timestamp=timestamp,
                    heading=self.waypoints[i].heading,
                    speed=speed
                )
        return None


@dataclass
class PathScore:
    """路径评分数据模型

    Requirements: 5.2, 5.3, 5.4
    """
    total: float
    safety: float
    efficiency: float
    compliance: float
    details: str = ""

    def __post_init__(self):
        for name, value in [
            ("total", self.total),
            ("safety", self.safety),
            ("efficiency", self.efficiency),
            ("compliance", self.compliance)
        ]:
            if not (0 <= value <= 1):
                raise ValueError(f"{name}评分必须在0到1之间: {value}")


@dataclass
class ControlCommand:
    """控制指令数据模型

    Requirements: 5.5
    """
    timestamp: float
    target_heading: Optional[float] = None
    target_speed: Optional[float] = None
    turn_rate: Optional[float] = None
    duration: float = 0.0
    command_type: str = ""

    def __post_init__(self):
        if self.timestamp < 0:
            raise ValueError(f"时间戳不能为负数: {self.timestamp}")
        if self.target_heading is not None:
            if not (0 <= self.target_heading < 360):
                raise ValueError(f"目标航向必须在0到360之间: {self.target_heading}")
        if self.target_speed is not None:
            if self.target_speed < 0:
                raise ValueError(f"目标速度不能为负数: {self.target_speed}")
        if self.turn_rate is not None:
            if not (-720 <= self.turn_rate <= 720):
                raise ValueError(f"转向率必须在-720到720度/分钟之间: {self.turn_rate}")
        if self.duration < 0:
            raise ValueError(f"持续时间不能为负数: {self.duration}")


@dataclass
class AvoidanceStrategy:
    """避让策略数据模型

    Requirements: 5.1-5.5
    """
    strategy_id: str
    path: Path
    score: PathScore
    control_commands: List[ControlCommand] = field(default_factory=list)
    is_selected: bool = False
    reason: str = ""

    def __post_init__(self):
        if not self.strategy_id or not self.strategy_id.strip():
            raise ValueError("策略ID不能为空")


@dataclass
class ReturnPathConfig:
    """返航路径配置数据模型

    Requirements: 5.6
    """
    original_heading: float
    original_speed: float
    return_threshold_distance: float = DEFAULT_RETURN_DISTANCE
    return_threshold_time: float = DEFAULT_RETURN_TIME
    smooth_return: bool = True

    def __post_init__(self):
        if not (0 <= self.original_heading < 360):
            raise ValueError(f"原始航向必须在0到360之间: {self.original_heading}")
        if self.original_speed < 0:
            raise ValueError(f"原始速度不能为负数: {self.original_speed}")
        if self.return_threshold_distance < 0:
            raise ValueError(f"返航距离阈值不能为负数: {self.return_threshold_distance}")
        if self.return_threshold_time < 0:
            raise ValueError(f"返航时间阈值不能为负数: {self.return_threshold_time}")


# ============================================================================
# 辅助函数
# ============================================================================

def _calc_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """计算两点之间的近似距离（海里）"""
    lat_diff = lat2 - lat1
    avg_lat = (lat1 + lat2) / 2.0
    lon_diff = (lon2 - lon1) * math.cos(math.radians(avg_lat))
    return math.sqrt(lat_diff ** 2 + lon_diff ** 2) * DEGREES_TO_NM


def calculate_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """计算两点之间的距离（海里）

    Args:
        lat1: 点1纬度
        lon1: 点1经度
        lat2: 点2纬度
        lon2: 点2经度

    Returns:
        距离（海里）
    """
    return _calc_distance(lat1, lon1, lat2, lon2)


def calculate_path_length(waypoints: List[Waypoint]) -> float:
    """计算路径总长度（海里）"""
    if len(waypoints) < 2:
        return 0.0
    total = 0.0
    for i in range(len(waypoints) - 1):
        total += _calc_distance(
            waypoints[i].latitude, waypoints[i].longitude,
            waypoints[i + 1].latitude, waypoints[i + 1].longitude
        )
    return total


def normalize_heading(angle: float) -> float:
    """将航向归一化到[0, 360)范围"""
    return angle % 360.0


def normalize_angle_signed(angle: float) -> float:
    """将角度归一化到(-180, 180]范围"""
    angle = angle % 360.0
    if angle > 180.0:
        angle -= 360.0
    return angle


def calculate_heading_change(from_heading: float, to_heading: float) -> float:
    """计算航向改变量（带符号），正值右转，负值左转"""
    change = to_heading - from_heading
    return normalize_angle_signed(change)


def _predict_position(lat: float, lon: float, heading: float,
                      speed_kts: float, dt_minutes: float):
    """根据航向速度预测dt_minutes分钟后的位置

    Returns:
        (new_lat, new_lon)
    """
    # speed_kts 单位：节（海里/小时）
    dist_nm = speed_kts * dt_minutes / 60.0
    # heading: 0=北, 90=东, 顺时针
    rad = math.radians(heading)
    delta_lat = dist_nm * math.cos(rad) / DEGREES_TO_NM
    cos_lat = math.cos(math.radians(lat))
    if cos_lat < 1e-9:
        delta_lon = 0.0
    else:
        delta_lon = dist_nm * math.sin(rad) / (DEGREES_TO_NM * cos_lat)
    return lat + delta_lat, lon + delta_lon


def _build_path_waypoints(own_lat: float, own_lon: float,
                          own_heading: float, own_speed: float,
                          new_heading: float, new_speed: float,
                          duration_minutes: float,
                          time_step: float = PATH_TIME_STEP) -> List[Waypoint]:
    """构建路径航点序列

    从当前位置出发，以 new_heading/new_speed 航行 duration_minutes 分钟，
    按 time_step 分钟步长生成航点列表。

    Args:
        own_lat, own_lon: 当前位置
        own_heading: 当前航向（仅用于记录起始点）
        own_speed: 当前速度（仅用于记录起始点）
        new_heading: 避让航向
        new_speed: 避让速度
        duration_minutes: 路径总时长（分钟）
        time_step: 时间步长（分钟）

    Returns:
        Waypoint 列表（至少2个点）
    """
    waypoints = []
    # 起始点（当前状态，时刻0）
    waypoints.append(Waypoint(
        latitude=own_lat,
        longitude=own_lon,
        timestamp=0.0,
        heading=own_heading,
        speed=own_speed
    ))

    lat, lon = own_lat, own_lon
    t = 0.0
    while t < duration_minutes - 1e-6:
        step = min(time_step, duration_minutes - t)
        lat, lon = _predict_position(lat, lon, new_heading, new_speed, step)
        t += step
        waypoints.append(Waypoint(
            latitude=lat,
            longitude=lon,
            timestamp=t * 60.0,   # 转换为秒
            heading=new_heading,
            speed=new_speed
        ))

    # 确保至少有2个航点
    if len(waypoints) < 2:
        lat2, lon2 = _predict_position(own_lat, own_lon, new_heading, new_speed, time_step)
        waypoints.append(Waypoint(
            latitude=lat2,
            longitude=lon2,
            timestamp=time_step * 60.0,
            heading=new_heading,
            speed=new_speed
        ))

    return waypoints


class _MockShipState:
    """内部用：模拟 ShipState 接口，用于路径上预测点的风险评估"""
    def __init__(self, latitude, longitude, heading, sog):
        self.latitude = latitude
        self.longitude = longitude
        self.heading = heading
        self.sog = sog
        rad = math.radians(heading)
        self.vx = sog * math.sin(rad)
        self.vy = sog * math.cos(rad)
        self.speed = sog


# ============================================================================
# 核心功能函数
# ============================================================================

def generate_avoidance_paths(own_ship,
                             target_ships: List,
                             environment,
                             colregs_action: AvoidanceAction) -> List[Path]:
    """生成候选避让路径

    根据当前态势和COLREGS规则，生成多条候选避让路径：
    1. 改变航向（右转15°, 30°, 45°）
    2. 减速避让（速度降至70%, 80%, 90%）
    3. 组合策略（转向+减速）
    4. 保持原航线（作为基准对比）

    Args:
        own_ship: 本船状态（ShipState对象）
        target_ships: 目标船舶列表
        environment: 环境配置（当前未使用，预留扩展）
        colregs_action: COLREGS规则要求的避让动作

    Returns:
        候选路径列表

    Requirements: 5.1
    """
    paths = []
    own_lat = own_ship.latitude
    own_lon = own_ship.longitude
    own_heading = own_ship.heading
    own_speed = own_ship.sog

    # 根据 COLREGS 动作确定转向方向符号
    # STARBOARD -> 右转（+角度），PORT -> 左转（-角度）
    if (colregs_action.action_type in (ActionType.COURSE_CHANGE, ActionType.COMBINED)
            and colregs_action.turn_direction is not None):
        turn_sign = 1.0 if colregs_action.turn_direction == TurnDirection.STARBOARD else -1.0
    else:
        turn_sign = 1.0  # 默认右转

    path_idx = 0

    # ---- 1. 纯转向路径 ----
    for angle in DEFAULT_TURN_ANGLES:
        delta = turn_sign * angle
        new_heading = normalize_heading(own_heading + delta)
        waypoints = _build_path_waypoints(
            own_lat, own_lon, own_heading, own_speed,
            new_heading, own_speed, PATH_EVALUATION_HORIZON
        )
        path_len = calculate_path_length(waypoints)
        path_idx += 1
        paths.append(Path(
            path_id=f"course_{path_idx}",
            path_type=PathType.COURSE_CHANGE,
            waypoints=waypoints,
            course_change=delta,
            speed_factor=1.0,
            estimated_duration=PATH_EVALUATION_HORIZON,
            path_length=path_len,
            description=f"转向{'右' if turn_sign > 0 else '左'}{angle}度"
        ))

    # ---- 2. 纯减速路径 ----
    for factor in DEFAULT_SPEED_FACTORS:
        new_speed = own_speed * factor
        waypoints = _build_path_waypoints(
            own_lat, own_lon, own_heading, own_speed,
            own_heading, new_speed, PATH_EVALUATION_HORIZON
        )
        path_len = calculate_path_length(waypoints)
        path_idx += 1
        paths.append(Path(
            path_id=f"speed_{path_idx}",
            path_type=PathType.SPEED_CHANGE,
            waypoints=waypoints,
            course_change=0.0,
            speed_factor=factor,
            estimated_duration=PATH_EVALUATION_HORIZON,
            path_length=path_len,
            description=f"减速至{int(factor*100)}%"
        ))

    # ---- 3. 组合策略：最小转向角 + 最大减速 ----
    combo_angle = DEFAULT_TURN_ANGLES[0] * turn_sign
    combo_heading = normalize_heading(own_heading + combo_angle)
    combo_speed = own_speed * DEFAULT_SPEED_FACTORS[0]
    waypoints = _build_path_waypoints(
        own_lat, own_lon, own_heading, own_speed,
        combo_heading, combo_speed, PATH_EVALUATION_HORIZON
    )
    path_len = calculate_path_length(waypoints)
    path_idx += 1
    paths.append(Path(
        path_id=f"combined_{path_idx}",
        path_type=PathType.COMBINED,
        waypoints=waypoints,
        course_change=combo_angle,
        speed_factor=DEFAULT_SPEED_FACTORS[0],
        estimated_duration=PATH_EVALUATION_HORIZON,
        path_length=path_len,
        description=f"组合：转向{combo_angle:+.0f}度并减速至{int(DEFAULT_SPEED_FACTORS[0]*100)}%"
    ))

    # ---- 4. 保持原航线（基准） ----
    waypoints = _build_path_waypoints(
        own_lat, own_lon, own_heading, own_speed,
        own_heading, own_speed, PATH_EVALUATION_HORIZON
    )
    path_len = calculate_path_length(waypoints)
    path_idx += 1
    paths.append(Path(
        path_id=f"maintain_{path_idx}",
        path_type=PathType.MAINTAIN,
        waypoints=waypoints,
        course_change=0.0,
        speed_factor=1.0,
        estimated_duration=PATH_EVALUATION_HORIZON,
        path_length=path_len,
        description="保持原航线"
    ))

    return paths


def check_path_safety(path: Path, target_ships: List) -> bool:
    """检查路径安全性

    沿路径按时间步长预测本船位置，计算与每艘目标船的 DCPA，
    若任意时刻最近会遇距离 < MIN_SAFE_DISTANCE，则认为不安全。
    跳过第一个起始航点（它反映当前状态，不是避让后的状态）。

    Args:
        path: 待检查的路径
        target_ships: 目标船舶列表

    Returns:
        True 表示安全，False 表示不安全

    Requirements: 5.3
    """
    if not target_ships:
        return True

    # 跳过第一个起始航点（index=0），从避让航向开始检查
    check_waypoints = path.waypoints[1:] if len(path.waypoints) > 1 else path.waypoints

    for wp in check_waypoints:
        own_mock = _MockShipState(
            latitude=wp.latitude,
            longitude=wp.longitude,
            heading=wp.heading,
            sog=wp.speed
        )
        for target in target_ships:
            dcpa, tcpa = calculate_dcpa_tcpa(own_mock, target)
            # 若 DCPA 小于安全距离且 TCPA > 0（未来会发生），则不安全
            if dcpa < MIN_SAFE_DISTANCE and tcpa > 0:
                return False
    return True


def check_colregs_compliance(path: Path, colregs_action: AvoidanceAction) -> bool:
    """检查路径COLREGS合规性

    验证路径是否符合 COLREGS 规则要求的避让方向和幅度。

    Args:
        path: 待检查的路径
        colregs_action: COLREGS规则要求的避让动作

    Returns:
        True 表示合规，False 表示不合规

    Requirements: 5.4
    """
    # 保持原航线：仅当 COLREGS 要求 MAINTAIN 或 NO_ACTION 时合规
    if path.path_type == PathType.MAINTAIN:
        return colregs_action.action_type in (ActionType.MAINTAIN, ActionType.NO_ACTION)

    # COLREGS 无需动作时，保持即合规，其他路径均合规（保守策略）
    if colregs_action.action_type == ActionType.NO_ACTION:
        return path.path_type == PathType.MAINTAIN

    # COLREGS 要求 MAINTAIN 时，只有 MAINTAIN 路径合规
    if colregs_action.action_type == ActionType.MAINTAIN:
        return path.path_type == PathType.MAINTAIN

    # COLREGS 要求转向（COURSE_CHANGE / COMBINED）
    if colregs_action.action_type in (ActionType.COURSE_CHANGE, ActionType.COMBINED):
        if path.path_type == PathType.SPEED_CHANGE:
            # 纯减速路径：视为部分合规（不违反方向要求）
            return True
        if path.course_change is None:
            return True
        required_dir = colregs_action.turn_direction
        if required_dir == TurnDirection.STARBOARD:
            return path.course_change >= 0
        elif required_dir == TurnDirection.PORT:
            return path.course_change <= 0
        return True

    # COLREGS 要求减速（SPEED_CHANGE）
    if colregs_action.action_type == ActionType.SPEED_CHANGE:
        if path.path_type in (PathType.SPEED_CHANGE, PathType.COMBINED):
            return (path.speed_factor is not None and path.speed_factor < 1.0)
        return True

    return True


def evaluate_path(path: Path,
                  target_ships: List,
                  colregs_action: AvoidanceAction) -> PathScore:
    """评估路径的安全性、效率和合规性

    评估维度：
    1. 安全性（50%）：检查是否引发新的碰撞风险
    2. 效率（30%）：航程增加和时间延误
    3. 合规性（20%）：是否符合COLREGS规则

    Args:
        path: 待评估的路径
        target_ships: 目标船舶列表
        colregs_action: COLREGS规则要求的避让动作

    Returns:
        路径评分

    Requirements: 5.2, 5.3, 5.4
    """
    details_parts = []

    # ---- 安全性评分 ----
    # 沿路径计算最小 DCPA，DCPA 越大越安全
    min_dcpa = float('inf')
    if target_ships:
        for wp in path.waypoints:
            own_mock = _MockShipState(
                latitude=wp.latitude,
                longitude=wp.longitude,
                heading=wp.heading,
                sog=wp.speed
            )
            for target in target_ships:
                dcpa, tcpa = calculate_dcpa_tcpa(own_mock, target)
                if tcpa >= 0 and dcpa < min_dcpa:
                    min_dcpa = dcpa

    if math.isinf(min_dcpa):
        min_dcpa = MIN_SAFE_DISTANCE * 3  # 无目标船，视为绝对安全

    # 安全性归一化：min_dcpa / (2 * MIN_SAFE_DISTANCE)，上限为1.0
    safety_score = min(min_dcpa / (2.0 * MIN_SAFE_DISTANCE), 1.0)
    details_parts.append(f"min_dcpa={min_dcpa:.2f}nm")

    # ---- 效率评分 ----
    # 效率：课程偏离越小、减速越小，效率越高
    course_penalty = 0.0
    if path.course_change is not None:
        # 归一化课程偏离到 [0,1]（MAX_COURSE_CHANGE 度时效率最低）
        course_penalty = min(abs(path.course_change) / MAX_COURSE_CHANGE, 1.0)

    speed_penalty = 0.0
    if path.speed_factor is not None and path.speed_factor < 1.0:
        # 速度降低越多，效率越低
        speed_penalty = 1.0 - path.speed_factor

    # 效率评分：两者平均后取反
    efficiency_score = 1.0 - (course_penalty + speed_penalty) / 2.0
    efficiency_score = max(0.0, efficiency_score)
    details_parts.append(f"course_pen={course_penalty:.2f},speed_pen={speed_penalty:.2f}")

    # ---- 合规性评分 ----
    is_compliant = check_colregs_compliance(path, colregs_action)
    compliance_score = 1.0 if is_compliant else 0.0
    details_parts.append(f"compliant={is_compliant}")

    # ---- 综合评分 ----
    total = (WEIGHT_SAFETY * safety_score
             + WEIGHT_EFFICIENCY * efficiency_score
             + WEIGHT_COMPLIANCE * compliance_score)
    total = max(0.0, min(1.0, total))

    return PathScore(
        total=total,
        safety=safety_score,
        efficiency=efficiency_score,
        compliance=compliance_score,
        details="; ".join(details_parts)
    )


def select_best_path(paths: List[Path],
                     scores: List[PathScore]) -> Tuple[Optional[Path], Optional[PathScore]]:
    """选择最优路径

    根据综合评分选择最优路径，不安全路径（safety_score=0）将被过滤。

    Args:
        paths: 候选路径列表
        scores: 对应的评分列表

    Returns:
        (最优路径, 最优评分)，如果没有合适路径则返回 (None, None)

    Requirements: 5.5
    """
    if not paths or not scores:
        return None, None

    # 过滤掉安全性为0的路径
    valid = [(p, s) for p, s in zip(paths, scores) if s.safety > 0.0]
    if not valid:
        # 所有路径均不安全，选总分最高的（次优）
        best_idx = max(range(len(scores)), key=lambda i: scores[i].total)
        return paths[best_idx], scores[best_idx]

    best_path, best_score = max(valid, key=lambda x: x[1].total)
    return best_path, best_score


def generate_control_commands(path: Path, own_ship) -> List[ControlCommand]:
    """生成控制指令

    根据选定的路径生成航向和航速调整指令。
    第一条指令在 t=0 发出，指定目标航向和速度。

    Args:
        path: 选定的路径
        own_ship: 本船状态

    Returns:
        控制指令列表

    Requirements: 5.5
    """
    commands = []
    if not path.waypoints:
        return commands

    start_wp = path.waypoints[0]
    # 第一个非起始点确定目标航向
    if len(path.waypoints) > 1:
        target_wp = path.waypoints[1]
    else:
        target_wp = start_wp

    target_heading = target_wp.heading
    target_speed = target_wp.speed

    # 计算转向率（假设10度/分钟的标准转向率）
    heading_change = (target_heading - own_ship.heading) % 360.0
    if heading_change > 180.0:
        heading_change -= 360.0
    turn_rate = heading_change / 1.0 if abs(heading_change) > 0.1 else 0.0  # 1分钟内完成转向
    turn_rate = max(-720.0, min(720.0, turn_rate))

    duration_sec = path.estimated_duration * 60.0

    if abs(heading_change) > 0.1 and abs(target_speed - own_ship.sog) > 0.1:
        cmd_type = "course_and_speed_change"
    elif abs(heading_change) > 0.1:
        cmd_type = "course_change"
    elif abs(target_speed - own_ship.sog) > 0.1:
        cmd_type = "speed_change"
    else:
        cmd_type = "maintain"

    commands.append(ControlCommand(
        timestamp=0.0,
        target_heading=target_heading,
        target_speed=target_speed,
        turn_rate=turn_rate if abs(turn_rate) > 0.01 else None,
        duration=duration_sec,
        command_type=cmd_type
    ))

    return commands


def plan_return_path(own_ship,
                     return_config: ReturnPathConfig,
                     target_ships: List) -> Optional[Path]:
    """规划返航路径

    避让完成后，规划返回原航线的路径。
    首先检查目标船是否已经足够远或 TCPA 足够长，
    若安全则生成平滑返回原航向的路径。

    Args:
        own_ship: 本船当前状态
        return_config: 返航配置
        target_ships: 目标船舶列表（用于安全检查）

    Returns:
        返航路径，如果尚不安全则返回 None

    Requirements: 5.6
    """
    # 检查是否满足返航条件
    for target in target_ships:
        dcpa, tcpa = calculate_dcpa_tcpa(own_ship, target)
        # 计算当前距离
        dx = (target.longitude - own_ship.longitude) * math.cos(
            math.radians((own_ship.latitude + target.latitude) / 2.0)) * DEGREES_TO_NM
        dy = (target.latitude - own_ship.latitude) * DEGREES_TO_NM
        current_dist = math.sqrt(dx ** 2 + dy ** 2)

        # 若目标船距离不够远或 TCPA 太小，不返航
        if (current_dist < return_config.return_threshold_distance
                and 0 < tcpa < return_config.return_threshold_time):
            return None

    # 安全条件满足，生成返回原航向的路径
    original_heading = return_config.original_heading
    original_speed = return_config.original_speed
    own_lat = own_ship.latitude
    own_lon = own_ship.longitude
    own_heading = own_ship.heading

    if return_config.smooth_return:
        # 平滑返航：分两段——先回到原航向，再稳定航行
        # 段1：过渡段（用 PATH_TIME_STEP * 5 分钟平滑转回）
        transition_minutes = 5.0
        waypoints = _build_path_waypoints(
            own_lat, own_lon, own_heading, own_ship.sog,
            original_heading, original_speed, transition_minutes
        )
        # 段2：稳定段（继续以原航向/速度行驶剩余时间）
        stable_minutes = PATH_EVALUATION_HORIZON - transition_minutes
        last_wp = waypoints[-1]
        stable_wps = _build_path_waypoints(
            last_wp.latitude, last_wp.longitude,
            original_heading, original_speed,
            original_heading, original_speed, stable_minutes
        )
        # 合并，调整时间戳
        time_offset = last_wp.timestamp
        for wp in stable_wps[1:]:   # 跳过重复的起点
            waypoints.append(Waypoint(
                latitude=wp.latitude,
                longitude=wp.longitude,
                timestamp=wp.timestamp + time_offset,
                heading=wp.heading,
                speed=wp.speed
            ))
    else:
        # 直接返航
        waypoints = _build_path_waypoints(
            own_lat, own_lon, own_heading, own_ship.sog,
            original_heading, original_speed, PATH_EVALUATION_HORIZON
        )

    path_len = calculate_path_length(waypoints)
    return Path(
        path_id="return_path",
        path_type=PathType.COURSE_CHANGE,
        waypoints=waypoints,
        course_change=calculate_heading_change(own_heading, original_heading),
        speed_factor=original_speed / own_ship.sog if own_ship.sog > 1e-6 else 1.0,
        estimated_duration=PATH_EVALUATION_HORIZON,
        path_length=path_len,
        description="返回原航线"
    )


def plan_avoidance(own_ship,
                   target_ships: List,
                   environment,
                   colregs_action: AvoidanceAction) -> Optional[AvoidanceStrategy]:
    """完整避让决策流程

    整合路径生成、评估、选择和控制指令生成，返回最优避让策略。

    Args:
        own_ship: 本船状态
        target_ships: 目标船舶列表
        environment: 环境配置
        colregs_action: COLREGS 规则要求的避让动作

    Returns:
        最优避让策略，若无可行路径则返回 None

    Requirements: 5.1-5.5
    """
    # 1. 生成候选路径
    paths = generate_avoidance_paths(own_ship, target_ships, environment, colregs_action)
    if not paths:
        return None

    # 2. 评估所有候选路径
    scores = [evaluate_path(p, target_ships, colregs_action) for p in paths]

    # 3. 选择最优路径
    best_path, best_score = select_best_path(paths, scores)
    if best_path is None:
        return None

    # 4. 生成控制指令
    commands = generate_control_commands(best_path, own_ship)

    # 5. 标记路径状态
    best_path.status = PathStatus.SAFE if best_score.safety > 0.5 else PathStatus.RISKY

    return AvoidanceStrategy(
        strategy_id=f"strategy_{best_path.path_id}",
        path=best_path,
        score=best_score,
        control_commands=commands,
        is_selected=True,
        reason=f"综合评分最高: total={best_score.total:.3f}, "
               f"safety={best_score.safety:.3f}, "
               f"efficiency={best_score.efficiency:.3f}, "
               f"compliance={best_score.compliance:.3f}"
    )
  