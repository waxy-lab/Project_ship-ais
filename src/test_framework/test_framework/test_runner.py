"""
测试运行器：场景运行和数据收集
Requirements: 6.1-6.6
"""
import math
import uuid
from typing import List, Optional

from scenario_generator.models import ShipState, ScenarioConfig, EnvironmentConfig
from collision_avoidance.risk_assessment import (
    assess_collision_risk, RISK_THRESHOLD_WARNING
)
from collision_avoidance.rules_engine import (
    apply_colregs_rule, analyze_encounter_situation, ActionType
)
from collision_avoidance.path_planning import (
    plan_avoidance, plan_return_path, ReturnPathConfig
)
from .metrics import (
    PerformanceMetrics, BatchTestReport, ScenarioResult,
    CollisionEvent, CourseChangeEvent, ShipTrackPoint
)

COLLISION_DISTANCE_NM = 0.05
NEAR_MISS_DISTANCE_NM = 0.3
SIM_STEP_SEC = 10.0
EARTH_RADIUS_NM = 3440.065


def _distance_nm(lat1, lon1, lat2, lon2):
    R = EARTH_RADIUS_NM
    lat1r, lon1r, lat2r, lon2r = map(math.radians, [lat1, lon1, lat2, lon2])
    dlat, dlon = lat2r - lat1r, lon2r - lon1r
    a = math.sin(dlat/2)**2 + math.cos(lat1r)*math.cos(lat2r)*math.sin(dlon/2)**2
    return R * 2 * math.asin(math.sqrt(min(a, 1.0)))


def _advance_ship(ship, dt_sec):
    speed_nm_s = ship.sog / 3600.0
    dist_nm = speed_nm_s * dt_sec
    hr = math.radians(ship.heading)
    dlat = dist_nm * math.cos(hr) / 60.0
    dlon = dist_nm * math.sin(hr) / (60.0 * math.cos(math.radians(ship.latitude)))
    rot_step = ship.rot / 60.0 * dt_sec
    return ShipState(
        mmsi=ship.mmsi,
        latitude=ship.latitude + dlat,
        longitude=ship.longitude + dlon,
        heading=(ship.heading + rot_step) % 360.0,
        sog=ship.sog, rot=ship.rot,
        timestamp=ship.timestamp + dt_sec,
    )


class ScenarioRunner:
    """单场景运行器 Requirements: 6.1-6.5"""

    def __init__(self, own_ship_index=0,
                 collision_distance_nm=COLLISION_DISTANCE_NM,
                 near_miss_distance_nm=NEAR_MISS_DISTANCE_NM,
                 step_sec=SIM_STEP_SEC):
        self.own_idx = own_ship_index
        self.col_dist = collision_distance_nm
        self.nm_dist = near_miss_distance_nm
        self.step_sec = step_sec

    def run(self, scenario: ScenarioConfig) -> PerformanceMetrics:
        metrics = PerformanceMetrics(
            scenario_id=scenario.scenario_id,
            scenario_type=scenario.scenario_type.value,
        )
        ships = [ShipState.from_dict(s.to_dict()) for s in scenario.ships]
        env = scenario.environment
        steps = int(scenario.duration / self.step_sec)
        own = ships[self.own_idx]
        orig_heading = own.heading
        orig_speed = own.sog
        start_lat, start_lon = own.latitude, own.longitude
        is_avoiding = False
        avoid_start_t = 0.0
        last_heading = own.heading
        total_dist = 0.0
        dcpa_samples = []
        min_dist = float('inf')
        return_cfg = ReturnPathConfig(
            original_heading=orig_heading, original_speed=orig_speed)
        for s in ships:
            metrics.ship_tracks[s.mmsi] = []
        elapsed = 0.0
        for step in range(steps):
            elapsed = step * self.step_sec
            own = ships[self.own_idx]
            targets = [s for s in ships if s.mmsi != own.mmsi]
            # 记录轨迹
            for s in ships:
                metrics.ship_tracks[s.mmsi].append(
                    ShipTrackPoint(elapsed, s.latitude, s.longitude,
                                   s.heading, s.sog))
            # 碰撞检测
            for t in targets:
                d = _distance_nm(own.latitude, own.longitude,
                                 t.latitude, t.longitude)
                min_dist = min(min_dist, d)
                if d < self.col_dist:
                    metrics.collision_events.append(CollisionEvent(
                        timestamp=elapsed, own_mmsi=own.mmsi,
                        target_mmsi=t.mmsi, dcpa=d, tcpa=0.0,
                        cri=1.0, distance=d, is_collision=True))
                    metrics.result = ScenarioResult.COLLISION
            if metrics.result == ScenarioResult.COLLISION:
                break
            # 风险评估
            risk_results = []
            for t in targets:
                try:
                    r = assess_collision_risk(own, t)
                    risk_results.append((t, r))
                    dcpa_samples.append(r.dcpa)
                    if r.dcpa < self.nm_dist and r.tcpa > 0:
                        metrics.near_miss_events.append(CollisionEvent(
                            timestamp=elapsed, own_mmsi=own.mmsi,
                            target_mmsi=t.mmsi, dcpa=r.dcpa,
                            tcpa=r.tcpa, cri=r.cri,
                            distance=r.distance, is_collision=False))
                except Exception:
                    pass
            high_risk = [(t, r) for t, r in risk_results
                         if r.cri >= RISK_THRESHOLD_WARNING]
            new_heading = own.heading
            if high_risk:
                high_risk.sort(key=lambda x: x[1].cri, reverse=True)
                primary_t, _ = high_risk[0]
                if not is_avoiding:
                    is_avoiding = True
                    avoid_start_t = elapsed
                try:
                    sit = analyze_encounter_situation(own, primary_t)
                    act = apply_colregs_rule(sit.encounter_type, own, primary_t)
                    metrics.total_decisions += 1
                    if act.action_type != ActionType.NO_ACTION:
                        metrics.compliant_decisions += 1
                    if act.action_type not in (ActionType.MAINTAIN, ActionType.NO_ACTION):
                        strat = plan_avoidance(
                            own, [t for t, _ in high_risk], env, act)
                        if strat and strat.control_commands:
                            cmd = strat.control_commands[0]
                            if cmd.target_heading is not None:
                                new_heading = cmd.target_heading
                            if cmd.target_speed is not None and cmd.target_speed >= 0:
                                ships[self.own_idx] = ShipState(
                                    mmsi=own.mmsi, latitude=own.latitude,
                                    longitude=own.longitude, heading=own.heading,
                                    sog=cmd.target_speed, rot=own.rot,
                                    timestamp=own.timestamp)
                                own = ships[self.own_idx]
                except Exception:
                    pass
            elif is_avoiding:
                try:
                    rp = plan_return_path(own, return_cfg, targets)
                    if rp is not None:
                        is_avoiding = False
                        metrics.avoidance_duration_sec += elapsed - avoid_start_t
                        new_heading = orig_heading
                except Exception:
                    pass
            # 记录航向改变
            hdiff = abs(((new_heading - last_heading + 180) % 360) - 180)
            if hdiff > 1.0:
                metrics.course_change_events.append(CourseChangeEvent(
                    timestamp=elapsed, original_heading=last_heading,
                    new_heading=new_heading,
                    change_angle=new_heading - last_heading,
                    reason="avoidance" if high_risk else "return",
                    is_colregs_compliant=True))
                last_heading = new_heading
            # 推算位置
            prev_lat, prev_lon = own.latitude, own.longitude
            ships[self.own_idx] = ShipState(
                mmsi=own.mmsi, latitude=own.latitude,
                longitude=own.longitude, heading=new_heading,
                sog=own.sog, rot=own.rot, timestamp=own.timestamp)
            ships = [_advance_ship(s, self.step_sec) for s in ships]
            own = ships[self.own_idx]
            total_dist += _distance_nm(prev_lat, prev_lon,
                                       own.latitude, own.longitude)
        # 汇总
        metrics.duration_sec = elapsed
        metrics.min_distance_nm = min_dist if math.isfinite(min_dist) else 0.0
        metrics.min_dcpa_nm = min(dcpa_samples) if dcpa_samples else 0.0
        metrics.avg_dcpa_nm = (sum(dcpa_samples)/len(dcpa_samples)
                               if dcpa_samples else 0.0)
        metrics.course_change_count = len(metrics.course_change_events)
        if metrics.course_change_events:
            angs = [abs(e.change_angle) for e in metrics.course_change_events]
            metrics.avg_course_change_deg = sum(angs) / len(angs)
            metrics.total_course_change_deg = sum(angs)
        metrics.actual_distance_nm = total_dist
        direct = _distance_nm(start_lat, start_lon, own.latitude, own.longitude)
        metrics.original_distance_nm = direct
        if direct > 0.01:
            metrics.distance_increase_pct = (total_dist - direct) / direct * 100
        if metrics.total_decisions > 0:
            metrics.colregs_compliance_rate = (
                metrics.compliant_decisions / metrics.total_decisions)
        if metrics.result != ScenarioResult.COLLISION:
            metrics.result = ScenarioResult.SUCCESS
        return metrics


class TestRunner:
    """批量测试运行器 Requirements: 6.1-6.6"""

    def __init__(self, own_ship_index=0, step_sec=SIM_STEP_SEC):
        self._runner = ScenarioRunner(
            own_ship_index=own_ship_index, step_sec=step_sec)

    def run_scenario(self, scenario: ScenarioConfig) -> PerformanceMetrics:
        """运行单个场景 Requirements: 6.1-6.5"""
        return self._runner.run(scenario)

    def run_batch(self, scenarios: List[ScenarioConfig],
                  report_id: Optional[str] = None) -> BatchTestReport:
        """批量运行场景 Requirements: 6.6"""
        report = BatchTestReport(
            report_id=report_id or str(uuid.uuid4())[:8],
            total_scenarios=len(scenarios),
        )
        for scenario in scenarios:
            try:
                m = self._runner.run(scenario)
            except Exception as e:
                m = PerformanceMetrics(
                    scenario_id=scenario.scenario_id,
                    scenario_type=scenario.scenario_type.value,
                    result=ScenarioResult.ERROR, notes=str(e))
            report.scenario_metrics.append(m)
        self._aggregate(report)
        return report

    def _aggregate(self, report: BatchTestReport):
        """聚合指标 Requirements: 6.1-6.6"""
        ml = report.scenario_metrics
        if not ml:
            return
        report.success_count = sum(
            1 for m in ml if m.result == ScenarioResult.SUCCESS)
        report.collision_count = sum(m.collision_count for m in ml)
        report.near_miss_count = sum(m.near_miss_count for m in ml)
        report.success_rate = report.success_count / len(ml)
        dcpas = [m.min_dcpa_nm for m in ml
                 if math.isfinite(m.min_dcpa_nm) and m.min_dcpa_nm > 0]
        report.avg_min_dcpa_nm = sum(dcpas)/len(dcpas) if dcpas else 0.0
        report.overall_min_dcpa_nm = min(dcpas) if dcpas else float('inf')
        report.avg_course_change_count = (
            sum(m.course_change_count for m in ml) / len(ml))
        angs = [m.avg_course_change_deg for m in ml if m.course_change_count > 0]
        report.avg_course_change_deg = sum(angs)/len(angs) if angs else 0.0
        report.avg_distance_increase_pct = (
            sum(m.distance_increase_pct for m in ml) / len(ml))
        report.avg_time_delay_sec = (
            sum(m.time_delay_sec for m in ml) / len(ml))
        rates = [m.colregs_compliance_rate for m in ml if m.total_decisions > 0]
        report.avg_colregs_compliance_rate = (
            sum(rates)/len(rates) if rates else 0.0)
        by_type = {}
        for m in ml:
            by_type.setdefault(m.scenario_type, [])
            by_type[m.scenario_type].append(
                1 if m.result == ScenarioResult.SUCCESS else 0)
        report.success_rate_by_type = {
            t: sum(v)/len(v) for t, v in by_type.items()}


class TestRunner:
    """
    批量测试运行器
    Requirements: 6.1-6.6
    """

    def __init__(self, own_ship_index=0, step_sec=SIM_STEP_SEC):
        self._runner = ScenarioRunner(
            own_ship_index=own_ship_index, step_sec=step_sec)

    def run_scenario(self, scenario: ScenarioConfig) -> PerformanceMetrics:
        """运行单个场景 Requirements: 6.1-6.5"""
        return self._runner.run(scenario)

    def run_batch(self, scenarios: List[ScenarioConfig],
                  report_id: Optional[str] = None) -> BatchTestReport:
        """批量运行场景，生成汇总报告 Requirements: 6.6"""
        report = BatchTestReport(
            report_id=report_id or str(uuid.uuid4())[:8],
            total_scenarios=len(scenarios),
        )
        for scenario in scenarios:
            try:
                m = self._runner.run(scenario)
            except Exception as e:
                m = PerformanceMetrics(
                    scenario_id=scenario.scenario_id,
                    scenario_type=scenario.scenario_type.value,
                    result=ScenarioResult.ERROR,
                    notes=str(e),
                )
            report.scenario_metrics.append(m)
        self._aggregate(report)
        return report

    def _aggregate(self, report: BatchTestReport) -> None:
        """聚合各场景指标 Requirements: 6.1-6.6"""
        ml = report.scenario_metrics
        if not ml:
            return
        # 6.1 成功率
        report.success_count = sum(
            1 for m in ml if m.result == ScenarioResult.SUCCESS)
        report.collision_count = sum(m.collision_count for m in ml)
        report.near_miss_count = sum(m.near_miss_count for m in ml)
        report.success_rate = report.success_count / len(ml)
        # 6.2 DCPA
        dcpas = [m.min_dcpa_nm for m in ml
                 if math.isfinite(m.min_dcpa_nm) and m.min_dcpa_nm > 0]
        report.avg_min_dcpa_nm = sum(dcpas)/len(dcpas) if dcpas else 0.0
        report.overall_min_dcpa_nm = min(dcpas) if dcpas else float('inf')
        # 6.3 航向改变
        report.avg_course_change_count = (
            sum(m.course_change_count for m in ml) / len(ml))
        angs = [m.avg_course_change_deg for m in ml
                if m.course_change_count > 0]
        report.avg_course_change_deg = sum(angs)/len(angs) if angs else 0.0
        # 6.4 航程
        report.avg_distance_increase_pct = (
            sum(m.distance_increase_pct for m in ml) / len(ml))
        report.avg_time_delay_sec = (
            sum(m.time_delay_sec for m in ml) / len(ml))
        # 6.5 COLREGS 遵守率
        rates = [m.colregs_compliance_rate for m in ml
                 if m.total_decisions > 0]
        report.avg_colregs_compliance_rate = (
            sum(rates)/len(rates) if rates else 0.0)
        # 按类型分组成功率
        by_type: dict = {}
        for m in ml:
            by_type.setdefault(m.scenario_type, [])
            by_type[m.scenario_type].append(
                1 if m.result == ScenarioResult.SUCCESS else 0)
        report.success_rate_by_type = {
            t: sum(v)/len(v) for t, v in by_type.items()
        }


class TestRunner:
    """批量测试运行器 Requirements: 6.1-6.6"""

    def __init__(self, own_ship_index=0, step_sec=SIM_STEP_SEC):
        self._runner = ScenarioRunner(
            own_ship_index=own_ship_index, step_sec=step_sec)

    def run_scenario(self, scenario):
        """运行单个场景"""
        return self._runner.run(scenario)

    def run_batch(self, scenarios, report_id=None):
        """批量运行场景 Requirements: 6.6"""
        report = BatchTestReport(
            report_id=report_id or str(uuid.uuid4())[:8],
            total_scenarios=len(scenarios),
        )
        for scenario in scenarios:
            try:
                m = self._runner.run(scenario)
            except Exception as e:
                m = PerformanceMetrics(
                    scenario_id=scenario.scenario_id,
                    scenario_type=scenario.scenario_type.value,
                    result=ScenarioResult.ERROR, notes=str(e))
            report.scenario_metrics.append(m)
        self._aggregate(report)
        return report

    def _aggregate(self, report):
        ml = report.scenario_metrics
        if not ml:
            return
        report.success_count = sum(
            1 for m in ml if m.result == ScenarioResult.SUCCESS)
        report.collision_count = sum(m.collision_count for m in ml)
        report.near_miss_count = sum(m.near_miss_count for m in ml)
        report.success_rate = report.success_count / len(ml)
        dcpas = [m.min_dcpa_nm for m in ml
                 if math.isfinite(m.min_dcpa_nm) and m.min_dcpa_nm > 0]
        report.avg_min_dcpa_nm = sum(dcpas)/len(dcpas) if dcpas else 0.0
        report.overall_min_dcpa_nm = min(dcpas) if dcpas else float('inf')
        report.avg_course_change_count = (
            sum(m.course_change_count for m in ml) / len(ml))
        angs = [m.avg_course_change_deg for m in ml if m.course_change_count > 0]
        report.avg_course_change_deg = sum(angs)/len(angs) if angs else 0.0
        report.avg_distance_increase_pct = (
            sum(m.distance_increase_pct for m in ml) / len(ml))
        report.avg_time_delay_sec = (
            sum(m.time_delay_sec for m in ml) / len(ml))
        rates = [m.colregs_compliance_rate for m in ml if m.total_decisions > 0]
        report.avg_colregs_compliance_rate = (
            sum(rates)/len(rates) if rates else 0.0)
        by_type = {}
        for m in ml:
            by_type.setdefault(m.scenario_type, [])
            by_type[m.scenario_type].append(
                1 if m.result == ScenarioResult.SUCCESS else 0)
        report.success_rate_by_type = {
            t: sum(v)/len(v) for t, v in by_type.items()}
