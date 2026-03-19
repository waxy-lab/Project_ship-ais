"""
性能指标数据模型

定义避碰算法性能评估的所有数据类和指标。
Requirements: 6.1-6.5
"""

from dataclasses import dataclass, field
from typing import List, Optional, Dict
from enum import Enum
import math


class ScenarioResult(Enum):
    """场景结果枚举"""
    SUCCESS = "success"          # 成功避碰
    COLLISION = "collision"      # 发生碰撞
    NEAR_MISS = "near_miss"      # 险些碰撞（DCPA < 安全距离但 > 0）
    TIMEOUT = "timeout"          # 超时未完成
    ERROR = "error"              # 运行错误


@dataclass
class CollisionEvent:
    """碰撞/险情事件记录"""
    timestamp: float             # 事件时间（秒）
    own_mmsi: int               # 本船 MMSI
    target_mmsi: int            # 目标船 MMSI
    dcpa: float                  # 最近会遇距离（海里）
    tcpa: float                  # 到达最近点时间（秒）
    cri: float                   # 碰撞风险指数
    distance: float              # 当前距离（海里）
    is_collision: bool           # 是否为碰撞（True）还是险情（False）


@dataclass
class CourseChangeEvent:
    """航向改变事件记录"""
    timestamp: float             # 事件时间（秒）
    original_heading: float      # 改变前航向（度）
    new_heading: float           # 改变后航向（度）
    change_angle: float          # 改变角度（度，正=右转）
    reason: str                  # 改变原因
    is_colregs_compliant: bool   # 是否符合 COLREGS


@dataclass
class ShipTrackPoint:
    """船舶轨迹点"""
    timestamp: float
    latitude: float
    longitude: float
    heading: float
    sog: float


@dataclass
class PerformanceMetrics:
    """
    避碰算法性能指标数据类

    Requirements:
        6.1 - 避碰成功率
        6.2 - 平均避让距离和最小会遇距离
        6.3 - 航向改变次数和平均改变角度
        6.4 - 航程增加百分比和时间延误
        6.5 - COLREGS 规则遵守率
    """

    # ---- 场景基础信息 ----
    scenario_id: str = ""
    scenario_type: str = ""       # head_on / crossing / overtaking / multi_ship
    duration_sec: float = 0.0    # 场景总时长（秒）

    # ---- Requirement 6.1: 避碰成功率 ----
    result: ScenarioResult = ScenarioResult.SUCCESS
    collision_events: List[CollisionEvent] = field(default_factory=list)
    near_miss_events: List[CollisionEvent] = field(default_factory=list)

    # ---- Requirement 6.2: 避让距离 ----
    min_dcpa_nm: float = float('inf')    # 最小 DCPA（海里）
    avg_dcpa_nm: float = 0.0             # 平均 DCPA（海里）
    min_distance_nm: float = float('inf') # 最小实际距离（海里）

    # ---- Requirement 6.3: 航向改变 ----
    course_change_count: int = 0         # 航向改变次数
    course_change_events: List[CourseChangeEvent] = field(default_factory=list)
    avg_course_change_deg: float = 0.0   # 平均改变角度（度）
    total_course_change_deg: float = 0.0 # 总改变角度（度）

    # ---- Requirement 6.4: 航程与时间 ----
    original_distance_nm: float = 0.0    # 原始直线距离（海里）
    actual_distance_nm: float = 0.0      # 实际航行距离（海里）
    distance_increase_pct: float = 0.0   # 航程增加百分比
    time_delay_sec: float = 0.0          # 时间延误（秒）
    avoidance_duration_sec: float = 0.0  # 避让持续时间（秒）

    # ---- Requirement 6.5: COLREGS 遵守率 ----
    total_decisions: int = 0             # 总决策次数
    compliant_decisions: int = 0         # 符合 COLREGS 的决策次数
    colregs_compliance_rate: float = 0.0 # COLREGS 遵守率（0-1）

    # ---- 附加信息 ----
    ship_tracks: Dict[int, List[ShipTrackPoint]] = field(default_factory=dict)  # MMSI -> 轨迹
    notes: str = ""

    @property
    def is_successful(self) -> bool:
        """场景是否成功（无碰撞）"""
        return self.result == ScenarioResult.SUCCESS

    @property
    def collision_count(self) -> int:
        """碰撞次数"""
        return len(self.collision_events)

    @property
    def near_miss_count(self) -> int:
        """险情次数"""
        return len(self.near_miss_events)

    def to_dict(self) -> dict:
        """转换为字典（用于 JSON 序列化和报告生成）"""
        return {
            "scenario_id": self.scenario_id,
            "scenario_type": self.scenario_type,
            "duration_sec": self.duration_sec,
            "result": self.result.value,
            "is_successful": self.is_successful,
            # 6.1
            "collision_count": self.collision_count,
            "near_miss_count": self.near_miss_count,
            # 6.2
            "min_dcpa_nm": round(self.min_dcpa_nm, 4) if math.isfinite(self.min_dcpa_nm) else None,
            "avg_dcpa_nm": round(self.avg_dcpa_nm, 4),
            "min_distance_nm": round(self.min_distance_nm, 4) if math.isfinite(self.min_distance_nm) else None,
            # 6.3
            "course_change_count": self.course_change_count,
            "avg_course_change_deg": round(self.avg_course_change_deg, 2),
            "total_course_change_deg": round(self.total_course_change_deg, 2),
            # 6.4
            "original_distance_nm": round(self.original_distance_nm, 4),
            "actual_distance_nm": round(self.actual_distance_nm, 4),
            "distance_increase_pct": round(self.distance_increase_pct, 2),
            "time_delay_sec": round(self.time_delay_sec, 1),
            "avoidance_duration_sec": round(self.avoidance_duration_sec, 1),
            # 6.5
            "total_decisions": self.total_decisions,
            "compliant_decisions": self.compliant_decisions,
            "colregs_compliance_rate": round(self.colregs_compliance_rate, 4),
            "notes": self.notes,
        }


@dataclass
class BatchTestReport:
    """
    批量测试报告

    聚合多个场景的 PerformanceMetrics，生成汇总统计。
    Requirements: 6.1-6.6
    """
    report_id: str = ""
    total_scenarios: int = 0
    scenario_metrics: List[PerformanceMetrics] = field(default_factory=list)

    # ---- 汇总统计 ----
    success_count: int = 0
    collision_count: int = 0
    near_miss_count: int = 0
    success_rate: float = 0.0            # Requirement 6.1

    avg_min_dcpa_nm: float = 0.0         # Requirement 6.2
    overall_min_dcpa_nm: float = float('inf')

    avg_course_change_count: float = 0.0  # Requirement 6.3
    avg_course_change_deg: float = 0.0

    avg_distance_increase_pct: float = 0.0  # Requirement 6.4
    avg_time_delay_sec: float = 0.0

    avg_colregs_compliance_rate: float = 0.0  # Requirement 6.5

    # 按场景类型分组的成功率
    success_rate_by_type: Dict[str, float] = field(default_factory=dict)

    def to_dict(self) -> dict:
        """转换为字典（用于报告和可视化）"""
        return {
            "report_id": self.report_id,
            "total_scenarios": self.total_scenarios,
            "success_count": self.success_count,
            "collision_count": self.collision_count,
            "near_miss_count": self.near_miss_count,
            "success_rate": round(self.success_rate, 4),
            "avg_min_dcpa_nm": round(self.avg_min_dcpa_nm, 4),
            "overall_min_dcpa_nm": round(self.overall_min_dcpa_nm, 4) if math.isfinite(self.overall_min_dcpa_nm) else None,
            "avg_course_change_count": round(self.avg_course_change_count, 2),
            "avg_course_change_deg": round(self.avg_course_change_deg, 2),
            "avg_distance_increase_pct": round(self.avg_distance_increase_pct, 2),
            "avg_time_delay_sec": round(self.avg_time_delay_sec, 1),
            "avg_colregs_compliance_rate": round(self.avg_colregs_compliance_rate, 4),
            "success_rate_by_type": {
                k: round(v, 4) for k, v in self.success_rate_by_type.items()
            },
        }
