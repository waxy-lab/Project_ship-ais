"""
性能指标计算模块
Requirements: 6.1-6.5
"""
import math
from typing import List, Dict, Tuple
from .metrics import (
    PerformanceMetrics, BatchTestReport, ScenarioResult
)

SAFE_DISTANCE_NM = 0.5


def calculate_success_rate(metrics_list: List[PerformanceMetrics]) -> float:
    """避碰成功率 Requirements: 6.1"""
    if not metrics_list:
        return 0.0
    return sum(1 for m in metrics_list if m.result == ScenarioResult.SUCCESS) / len(metrics_list)


def calculate_success_rate_by_type(metrics_list: List[PerformanceMetrics]) -> Dict[str, float]:
    """按场景类型计算成功率 Requirements: 6.1"""
    by_type: Dict[str, list] = {}
    for m in metrics_list:
        by_type.setdefault(m.scenario_type, [])
        by_type[m.scenario_type].append(1 if m.result == ScenarioResult.SUCCESS else 0)
    return {t: sum(v)/len(v) for t, v in by_type.items()}


def calculate_avg_dcpa(metrics_list: List[PerformanceMetrics]) -> float:
    """平均最小DCPA Requirements: 6.2"""
    values = [m.min_dcpa_nm for m in metrics_list
              if math.isfinite(m.min_dcpa_nm) and m.min_dcpa_nm > 0]
    return sum(values)/len(values) if values else 0.0


def calculate_overall_min_dcpa(metrics_list: List[PerformanceMetrics]) -> float:
    """所有场景最小会遇距离 Requirements: 6.2"""
    values = [m.min_dcpa_nm for m in metrics_list
              if math.isfinite(m.min_dcpa_nm) and m.min_dcpa_nm > 0]
    return min(values) if values else float('inf')


def calculate_course_change_stats(metrics: PerformanceMetrics) -> Tuple[int, float, float]:
    """航向改变统计 Requirements: 6.3
    Returns: (次数, 平均角度, 总角度)
    """
    events = metrics.course_change_events
    if not events:
        return 0, 0.0, 0.0
    angles = [abs(e.change_angle) for e in events]
    return len(events), sum(angles)/len(angles), sum(angles)


def calculate_avg_course_change_count(metrics_list: List[PerformanceMetrics]) -> float:
    """平均航向改变次数 Requirements: 6.3"""
    if not metrics_list:
        return 0.0
    return sum(m.course_change_count for m in metrics_list) / len(metrics_list)


def calculate_avg_course_change_angle(metrics_list: List[PerformanceMetrics]) -> float:
    """平均航向改变角度 Requirements: 6.3"""
    values = [m.avg_course_change_deg for m in metrics_list if m.course_change_count > 0]
    return sum(values)/len(values) if values else 0.0


def calculate_distance_increase(original_nm: float, actual_nm: float) -> float:
    """航程增加百分比 Requirements: 6.4"""
    if original_nm <= 0.0:
        return 0.0
    return max(0.0, (actual_nm - original_nm) / original_nm * 100.0)


def calculate_avg_distance_increase(metrics_list: List[PerformanceMetrics]) -> float:
    """平均航程增加百分比 Requirements: 6.4"""
    if not metrics_list:
        return 0.0
    return sum(m.distance_increase_pct for m in metrics_list) / len(metrics_list)


def calculate_colregs_compliance_rate(metrics: PerformanceMetrics) -> float:
    """单场景COLREGS遵守率 Requirements: 6.5"""
    if metrics.total_decisions == 0:
        return 1.0
    return metrics.compliant_decisions / metrics.total_decisions


def calculate_avg_colregs_compliance_rate(metrics_list: List[PerformanceMetrics]) -> float:
    """平均COLREGS遵守率 Requirements: 6.5"""
    rates = [m.colregs_compliance_rate for m in metrics_list if m.total_decisions > 0]
    return sum(rates)/len(rates) if rates else 1.0


def update_metrics_calculations(metrics: PerformanceMetrics) -> PerformanceMetrics:
    """重新计算所有派生指标 Requirements: 6.1-6.5"""
    count, avg_ang, total_ang = calculate_course_change_stats(metrics)
    metrics.course_change_count = count
    metrics.avg_course_change_deg = avg_ang
    metrics.total_course_change_deg = total_ang
    metrics.distance_increase_pct = calculate_distance_increase(
        metrics.original_distance_nm, metrics.actual_distance_nm)
    metrics.colregs_compliance_rate = calculate_colregs_compliance_rate(metrics)
    if metrics.collision_count > 0:
        metrics.result = ScenarioResult.COLLISION
    elif metrics.result != ScenarioResult.ERROR:
        metrics.result = ScenarioResult.SUCCESS
    return metrics


def compute_batch_report(metrics_list: List[PerformanceMetrics],
                         report_id: str = "") -> BatchTestReport:
    """从场景指标列表计算完整批量报告 Requirements: 6.1-6.6"""
    report = BatchTestReport(
        report_id=report_id,
        total_scenarios=len(metrics_list),
        scenario_metrics=list(metrics_list),
    )
    if not metrics_list:
        return report
    report.success_count = sum(1 for m in metrics_list if m.result == ScenarioResult.SUCCESS)
    report.collision_count = sum(m.collision_count for m in metrics_list)
    report.near_miss_count = sum(m.near_miss_count for m in metrics_list)
    report.success_rate = calculate_success_rate(metrics_list)
    report.success_rate_by_type = calculate_success_rate_by_type(metrics_list)
    report.avg_min_dcpa_nm = calculate_avg_dcpa(metrics_list)
    report.overall_min_dcpa_nm = calculate_overall_min_dcpa(metrics_list)
    report.avg_course_change_count = calculate_avg_course_change_count(metrics_list)
    report.avg_course_change_deg = calculate_avg_course_change_angle(metrics_list)
    report.avg_distance_increase_pct = calculate_avg_distance_increase(metrics_list)
    report.avg_time_delay_sec = sum(m.time_delay_sec for m in metrics_list) / len(metrics_list)
    report.avg_colregs_compliance_rate = calculate_avg_colregs_compliance_rate(metrics_list)
    return report
