"""
性能指标计算单元测试
Requirements: 6.1-6.5
"""
import math
import pytest

from test_framework.test_framework.metrics import (
    PerformanceMetrics, BatchTestReport, ScenarioResult,
    CollisionEvent, CourseChangeEvent
)
from test_framework.test_framework.metrics_calculator import (
    calculate_success_rate, calculate_success_rate_by_type,
    calculate_avg_dcpa, calculate_overall_min_dcpa,
    calculate_course_change_stats, calculate_avg_course_change_count,
    calculate_avg_course_change_angle, calculate_distance_increase,
    calculate_avg_distance_increase, calculate_colregs_compliance_rate,
    calculate_avg_colregs_compliance_rate, update_metrics_calculations,
    compute_batch_report,
)

def make_metrics(result=ScenarioResult.SUCCESS, scenario_type="head_on",
                 min_dcpa=1.0, course_changes=0, avg_change=0.0,
                 orig_dist=10.0, actual_dist=10.0,
                 total_dec=0, compliant_dec=0):
    m = PerformanceMetrics(
        scenario_id="test", scenario_type=scenario_type, result=result,
        min_dcpa_nm=min_dcpa, course_change_count=course_changes,
        avg_course_change_deg=avg_change, original_distance_nm=orig_dist,
        actual_distance_nm=actual_dist,
        distance_increase_pct=((actual_dist-orig_dist)/orig_dist*100 if orig_dist>0 else 0),
        total_decisions=total_dec, compliant_decisions=compliant_dec,
        colregs_compliance_rate=(compliant_dec/total_dec if total_dec>0 else 1.0),
    )
    return m

def make_event(angle):
    return CourseChangeEvent(0.0, 90.0, 90.0+angle, angle, "avoidance", True)

class TestSuccessRate:
    def test_all_success(self):
        assert calculate_success_rate([make_metrics(ScenarioResult.SUCCESS)]*5) == 1.0
    def test_all_failure(self):
        assert calculate_success_rate([make_metrics(ScenarioResult.COLLISION)]*3) == 0.0
    def test_half_success(self):
        ml = [make_metrics(ScenarioResult.SUCCESS)]*4 + [make_metrics(ScenarioResult.COLLISION)]*4
        assert calculate_success_rate(ml) == 0.5
    def test_empty_list(self):
        assert calculate_success_rate([]) == 0.0
    def test_success_rate_by_type(self):
        ml = [make_metrics(ScenarioResult.SUCCESS,"head_on"),
              make_metrics(ScenarioResult.SUCCESS,"head_on"),
              make_metrics(ScenarioResult.COLLISION,"head_on"),
              make_metrics(ScenarioResult.SUCCESS,"crossing")]
        rates = calculate_success_rate_by_type(ml)
        assert abs(rates["head_on"]-2/3) < 1e-9
        assert rates["crossing"] == 1.0
    def test_by_type_empty(self):
        assert calculate_success_rate_by_type([]) == {}

class TestDCPACalculation:
    def test_avg_dcpa_basic(self):
        ml = [make_metrics(min_dcpa=1.0), make_metrics(min_dcpa=3.0)]
        assert abs(calculate_avg_dcpa(ml)-2.0) < 1e-9
    def test_avg_dcpa_empty(self):
        assert calculate_avg_dcpa([]) == 0.0
    def test_avg_dcpa_filters_zero(self):
        ml = [make_metrics(min_dcpa=0.0), make_metrics(min_dcpa=2.0)]
        assert abs(calculate_avg_dcpa(ml)-2.0) < 1e-9
    def test_avg_dcpa_filters_inf(self):
        ml = [make_metrics(min_dcpa=float('inf')), make_metrics(min_dcpa=2.0)]
        assert abs(calculate_avg_dcpa(ml)-2.0) < 1e-9
    def test_overall_min_dcpa(self):
        ml = [make_metrics(min_dcpa=1.5), make_metrics(min_dcpa=0.8), make_metrics(min_dcpa=2.0)]
        assert abs(calculate_overall_min_dcpa(ml)-0.8) < 1e-9
    def test_overall_min_dcpa_empty(self):
        assert calculate_overall_min_dcpa([]) == float('inf')
    def test_larger_dcpa_safer(self):
        assert calculate_avg_dcpa([make_metrics(min_dcpa=2.0)]) > calculate_avg_dcpa([make_metrics(min_dcpa=0.3)])

class TestCourseChangeStats:
    def test_no_changes(self):
        c,a,t = calculate_course_change_stats(make_metrics())
        assert c==0 and a==0.0 and t==0.0
    def test_single_change(self):
        m = make_metrics(); m.course_change_events = [make_event(30.0)]
        c,a,t = calculate_course_change_stats(m)
        assert c==1 and abs(a-30.0)<1e-9 and abs(t-30.0)<1e-9
    def test_multiple_changes(self):
        m = make_metrics()
        m.course_change_events = [make_event(20.0), make_event(-30.0), make_event(10.0)]
        c,a,t = calculate_course_change_stats(m)
        assert c==3 and abs(a-20.0)<1e-9 and abs(t-60.0)<1e-9
    def test_avg_count(self):
        ml = [make_metrics(course_changes=4), make_metrics(course_changes=2)]
        assert abs(calculate_avg_course_change_count(ml)-3.0)<1e-9
    def test_avg_count_empty(self):
        assert calculate_avg_course_change_count([]) == 0.0
    def test_avg_angle(self):
        ml = [make_metrics(course_changes=1,avg_change=30.0),
              make_metrics(course_changes=2,avg_change=10.0)]
        assert abs(calculate_avg_course_change_angle(ml)-20.0)<1e-9
    def test_avg_angle_skips_zero_count(self):
        ml = [make_metrics(course_changes=0,avg_change=50.0),
              make_metrics(course_changes=1,avg_change=20.0)]
        assert abs(calculate_avg_course_change_angle(ml)-20.0)<1e-9

class TestDistanceIncrease:
    def test_no_increase(self):
        assert calculate_distance_increase(10.0,10.0) == 0.0
    def test_ten_percent(self):
        assert abs(calculate_distance_increase(10.0,11.0)-10.0)<1e-9
    def test_zero_original(self):
        assert calculate_distance_increase(0.0,5.0) == 0.0
    def test_negative_clamped(self):
        assert calculate_distance_increase(10.0,9.0) == 0.0
    def test_avg_increase(self):
        ml = [make_metrics(orig_dist=10.0,actual_dist=11.0),
              make_metrics(orig_dist=10.0,actual_dist=12.0)]
        assert abs(calculate_avg_distance_increase(ml)-15.0)<1e-6
    def test_avg_increase_empty(self):
        assert calculate_avg_distance_increase([]) == 0.0
    def test_more_avoidance_more_distance(self):
        assert calculate_distance_increase(10.0,10.5) < calculate_distance_increase(10.0,12.0)

class TestColregsCompliance:
    def test_full_compliance(self):
        assert calculate_colregs_compliance_rate(make_metrics(total_dec=10,compliant_dec=10)) == 1.0
    def test_no_decisions(self):
        assert calculate_colregs_compliance_rate(make_metrics(total_dec=0)) == 1.0
    def test_partial(self):
        assert abs(calculate_colregs_compliance_rate(make_metrics(total_dec=10,compliant_dec=8))-0.8)<1e-9
    def test_zero_compliance(self):
        assert calculate_colregs_compliance_rate(make_metrics(total_dec=5,compliant_dec=0)) == 0.0
    def test_avg(self):
        ml = [make_metrics(total_dec=10,compliant_dec=10), make_metrics(total_dec=10,compliant_dec=6)]
        assert abs(calculate_avg_colregs_compliance_rate(ml)-0.8)<1e-9
    def test_avg_skips_no_decision(self):
        ml = [make_metrics(total_dec=0), make_metrics(total_dec=10,compliant_dec=8)]
        assert abs(calculate_avg_colregs_compliance_rate(ml)-0.8)<1e-9
    def test_avg_all_no_decision(self):
        assert calculate_avg_colregs_compliance_rate([make_metrics(total_dec=0)]*2) == 1.0

class TestUpdateMetrics:
    def test_course_change_stats(self):
        m = make_metrics()
        m.course_change_events = [make_event(30.0), make_event(10.0)]
        r = update_metrics_calculations(m)
        assert r.course_change_count==2 and abs(r.avg_course_change_deg-20.0)<1e-9
    def test_distance_increase(self):
        m = make_metrics(orig_dist=10.0, actual_dist=11.0)
        assert abs(update_metrics_calculations(m).distance_increase_pct-10.0)<1e-9
    def test_compliance_rate(self):
        m = make_metrics(total_dec=4, compliant_dec=3)
        assert abs(update_metrics_calculations(m).colregs_compliance_rate-0.75)<1e-9
    def test_collision_sets_result(self):
        m = make_metrics(result=ScenarioResult.SUCCESS)
        m.collision_events.append(CollisionEvent(0.0,111111111,222222222,0.01,0.0,1.0,0.01,True))
        assert update_metrics_calculations(m).result == ScenarioResult.COLLISION

class TestComputeBatchReport:
    def test_basic(self):
        ml = [make_metrics(ScenarioResult.SUCCESS,total_dec=10,compliant_dec=10),
              make_metrics(ScenarioResult.COLLISION,total_dec=5,compliant_dec=3)]
        r = compute_batch_report(ml,"r001")
        assert r.report_id=="r001" and r.total_scenarios==2
        assert r.success_count==1 and abs(r.success_rate-0.5)<1e-9
    def test_empty(self):
        r = compute_batch_report([])
        assert r.total_scenarios==0 and r.success_rate==0.0
    def test_all_success(self):
        assert compute_batch_report([make_metrics(ScenarioResult.SUCCESS)]*4).success_rate == 1.0
    def test_compliance_aggregated(self):
        ml = [make_metrics(total_dec=10,compliant_dec=10), make_metrics(total_dec=10,compliant_dec=8)]
        assert abs(compute_batch_report(ml).avg_colregs_compliance_rate-0.9)<1e-9
    def test_by_type_in_report(self):
        ml = [make_metrics(ScenarioResult.SUCCESS,"head_on"),
              make_metrics(ScenarioResult.COLLISION,"head_on"),
              make_metrics(ScenarioResult.SUCCESS,"crossing")]
        r = compute_batch_report(ml)
        assert abs(r.success_rate_by_type["head_on"]-0.5)<1e-9
        assert r.success_rate_by_type["crossing"]==1.0

if __name__ == '__main__':
    pytest.main([__file__, '-v'])
