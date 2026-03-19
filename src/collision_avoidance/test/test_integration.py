"""
集成测试：完整避碰决策流程
Requirements: 1.1-1.5, 3.1-3.6, 4.1-4.5, 5.1-5.6
"""
import pytest
import math
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../scenario_generator'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../test_framework'))

from scenario_generator.models import ShipState, EnvironmentConfig
from collision_avoidance.risk_assessment import (
    assess_collision_risk, RiskLevel, RISK_THRESHOLD_WARNING
)
from collision_avoidance.rules_engine import (
    apply_colregs_rule, analyze_encounter_situation,
    ActionType, TurnDirection, EncounterType
)
from collision_avoidance.path_planning import (
    plan_avoidance, plan_return_path, ReturnPathConfig,
    PathType, AvoidanceStrategy, MIN_SAFE_DISTANCE
)


def make_ship(**kwargs):
    defaults = dict(mmsi=123456789, latitude=30.0, longitude=120.0,
                    heading=90.0, sog=10.0, rot=0.0)
    defaults.update(kwargs)
    return ShipState(**defaults)


def nm_lon(nm, lat=30.0):
    return nm / (60.0 * math.cos(math.radians(lat)))


def nm_lat(nm):
    return nm / 60.0


# ============================================================================
# 场景1：对遇场景完整流程  Requirements: 1.1, 3.1-3.6, 4.1-4.5, 5.1-5.5
# ============================================================================

class TestHeadOnScenarioIntegration:

    def setup_method(self):
        self.own = make_ship(mmsi=123456789, latitude=30.0, longitude=120.0,
                             heading=90.0, sog=15.0)
        self.target = make_ship(mmsi=987654321, latitude=30.0,
                                longitude=120.0 + nm_lon(10.0),
                                heading=270.0, sog=15.0)
        self.env = EnvironmentConfig()

    def test_step1_risk_detects_danger(self):
        """风险评估应检测到对遇危险 Requirements: 3.1-3.3"""
        risk = assess_collision_risk(self.own, self.target)
        assert risk.cri >= RISK_THRESHOLD_WARNING
        assert risk.dcpa < MIN_SAFE_DISTANCE
        assert risk.tcpa > 0

    def test_step2_encounter_head_on(self):
        """态势分析应识别为对遇 Requirements: 4.1"""
        situation = analyze_encounter_situation(self.own, self.target)
        assert situation.encounter_type == EncounterType.HEAD_ON

    def test_step3_colregs_starboard(self):
        """COLREGS Rule 14 要求右转 Requirements: 4.2-4.5"""
        situation = analyze_encounter_situation(self.own, self.target)
        action = apply_colregs_rule(situation.encounter_type, self.own, self.target)
        assert action.action_type == ActionType.COURSE_CHANGE
        assert action.turn_direction == TurnDirection.STARBOARD

    def test_step4_strategy_generated(self):
        """路径规划应生成策略 Requirements: 5.1-5.5"""
        situation = analyze_encounter_situation(self.own, self.target)
        action = apply_colregs_rule(situation.encounter_type, self.own, self.target)
        strategy = plan_avoidance(self.own, [self.target], self.env, action)
        assert strategy is not None
        assert strategy.is_selected
        assert len(strategy.control_commands) >= 1

    def test_step5_strategy_compliant(self):
        """策略应符合COLREGS Requirements: 5.4"""
        situation = analyze_encounter_situation(self.own, self.target)
        action = apply_colregs_rule(situation.encounter_type, self.own, self.target)
        strategy = plan_avoidance(self.own, [self.target], self.env, action)
        assert strategy is not None
        assert strategy.score.compliance > 0.0
        if strategy.path.path_type == PathType.COURSE_CHANGE:
            assert strategy.path.course_change >= 0

    def test_step6_score_valid(self):
        """策略评分应有效 Requirements: 5.2-5.3"""
        situation = analyze_encounter_situation(self.own, self.target)
        action = apply_colregs_rule(situation.encounter_type, self.own, self.target)
        strategy = plan_avoidance(self.own, [self.target], self.env, action)
        assert strategy is not None
        assert 0 <= strategy.score.total <= 1.0
        assert strategy.score.safety > 0.0

    def test_step7_control_commands_valid(self):
        """控制指令应有效 Requirements: 5.5"""
        situation = analyze_encounter_situation(self.own, self.target)
        action = apply_colregs_rule(situation.encounter_type, self.own, self.target)
        strategy = plan_avoidance(self.own, [self.target], self.env, action)
        assert strategy is not None
        cmd = strategy.control_commands[0]
        assert cmd.timestamp == 0.0
        if cmd.target_heading is not None:
            assert 0 <= cmd.target_heading < 360
        if cmd.target_speed is not None:
            assert cmd.target_speed >= 0

    def test_full_pipeline(self):
        """完整对遇流程端到端 Requirements: 3.1-3.6, 4.1-4.5, 5.1-5.5"""
        risk = assess_collision_risk(self.own, self.target)
        assert risk.cri >= RISK_THRESHOLD_WARNING
        situation = analyze_encounter_situation(self.own, self.target)
        assert situation.encounter_type == EncounterType.HEAD_ON
        action = apply_colregs_rule(situation.encounter_type, self.own, self.target)
        assert action.action_type != ActionType.NO_ACTION
        strategy = plan_avoidance(self.own, [self.target], self.env, action)
        assert strategy is not None
        assert len(strategy.control_commands) >= 1


class TestOvertakingScenarioIntegration:

    def setup_method(self):
        self.own = make_ship(mmsi=123456789, latitude=30.0, longitude=120.0,
                             heading=90.0, sog=20.0)
        self.target = make_ship(mmsi=987654321, latitude=30.0,
                                longitude=120.0 + nm_lon(3.0),
                                heading=90.0, sog=8.0)
        self.env = EnvironmentConfig()

    def test_risk_detected(self):
        risk = assess_collision_risk(self.own, self.target)
        assert risk.cri > 0.0

    def test_encounter_type_overtaking(self):
        situation = analyze_encounter_situation(self.own, self.target)
        # 追越场景：距离3海里、本船快得多，应识别为OVERTAKING或判定CRI有效
        assert situation.encounter_type in (
            EncounterType.OVERTAKING, EncounterType.NONE
        ), f"追越场景encounter_type={situation.encounter_type}"

    def test_full_pipeline_overtaking(self):
        situation = analyze_encounter_situation(self.own, self.target)
        action = apply_colregs_rule(situation.encounter_type, self.own, self.target)
        if action.action_type not in (ActionType.MAINTAIN, ActionType.NO_ACTION):
            strategy = plan_avoidance(self.own, [self.target], self.env, action)
            assert strategy is not None


class TestCrossingScenarioIntegration:

    def setup_method(self):
        self.own = make_ship(mmsi=123456789, latitude=30.0, longitude=120.0,
                             heading=0.0, sog=12.0)
        self.target = make_ship(mmsi=987654321,
                                latitude=30.0 + nm_lat(5.0),
                                longitude=120.0 + nm_lon(3.0),
                                heading=270.0, sog=12.0)
        self.env = EnvironmentConfig()

    def test_risk_detected(self):
        risk = assess_collision_risk(self.own, self.target)
        assert risk.cri > 0.0

    def test_encounter_type_crossing(self):
        situation = analyze_encounter_situation(self.own, self.target)
        assert situation.encounter_type == EncounterType.CROSSING

    def test_full_pipeline_crossing(self):
        situation = analyze_encounter_situation(self.own, self.target)
        action = apply_colregs_rule(situation.encounter_type, self.own, self.target)
        if action.action_type not in (ActionType.MAINTAIN, ActionType.NO_ACTION):
            strategy = plan_avoidance(self.own, [self.target], self.env, action)
            assert strategy is not None
            assert strategy.score.total > 0.0


class TestMultiThreatScenarioIntegration:

    def setup_method(self):
        self.own = make_ship(mmsi=123456789, latitude=30.0, longitude=120.0,
                             heading=90.0, sog=12.0)
        self.target1 = make_ship(mmsi=111111111, latitude=30.0,
                                 longitude=120.0 + nm_lon(8.0),
                                 heading=270.0, sog=12.0)
        self.target2 = make_ship(mmsi=222222222,
                                 latitude=30.0 + nm_lat(4.0),
                                 longitude=120.0 + nm_lon(4.0),
                                 heading=270.0, sog=10.0)
        self.env = EnvironmentConfig()

    def test_both_threats_assessed(self):
        risk1 = assess_collision_risk(self.own, self.target1)
        risk2 = assess_collision_risk(self.own, self.target2)
        assert risk1.cri > 0.0
        assert risk2.cri > 0.0

    def test_highest_risk_prioritized(self):
        risk1 = assess_collision_risk(self.own, self.target1)
        risk2 = assess_collision_risk(self.own, self.target2)
        risks = sorted([risk1, risk2], key=lambda x: x.cri, reverse=True)
        assert risks[0].cri >= risks[1].cri

    def test_multi_threat_avoidance(self):
        situation = analyze_encounter_situation(self.own, self.target1)
        action = apply_colregs_rule(situation.encounter_type, self.own, self.target1)
        if action.action_type not in (ActionType.MAINTAIN, ActionType.NO_ACTION):
            strategy = plan_avoidance(
                self.own, [self.target1, self.target2], self.env, action)
            assert strategy is not None


class TestSafeScenarioIntegration:

    def test_distant_ships_no_risk(self):
        own = make_ship(mmsi=123456789, latitude=30.0, longitude=120.0,
                        heading=90.0, sog=10.0)
        # 目标船在100海里右侧，平行同向，不会相遇
        target = make_ship(mmsi=987654321,
                           latitude=30.0 + nm_lat(100.0),
                           longitude=120.0,
                           heading=90.0, sog=10.0)
        risk = assess_collision_risk(own, target)
        assert risk.cri < RISK_THRESHOLD_WARNING, \
            f"平行同向100海里船不应触发预警，CRI={risk.cri:.3f}"

    def test_parallel_ships_safe_dcpa(self):
        own = make_ship(mmsi=123456789, latitude=30.0, longitude=120.0,
                        heading=90.0, sog=12.0)
        target = make_ship(mmsi=987654321, latitude=30.0 + nm_lat(3.0),
                           longitude=120.0, heading=90.0, sog=12.0)
        risk = assess_collision_risk(own, target)
        assert risk.dcpa >= 2.0


class TestReturnPathIntegration:

    def test_return_after_avoidance(self):
        own = make_ship(mmsi=123456789, latitude=30.0,
                        longitude=120.0 + nm_lon(5.0), heading=105.0, sog=15.0)
        target = make_ship(mmsi=987654321, latitude=30.0,
                           longitude=120.0 + nm_lon(20.0), heading=270.0, sog=15.0)
        cfg = ReturnPathConfig(original_heading=90.0, original_speed=15.0,
                               return_threshold_distance=2.0, return_threshold_time=5.0)
        result = plan_return_path(own, cfg, [target])
        assert result is not None
        assert result.path_id == "return_path"

    def test_no_return_when_threat_close(self):
        own = make_ship(mmsi=123456789, latitude=30.0, longitude=120.0,
                        heading=105.0, sog=15.0)
        target = make_ship(mmsi=987654321, latitude=30.0,
                           longitude=120.0 + nm_lon(0.5), heading=270.0, sog=15.0)
        cfg = ReturnPathConfig(original_heading=90.0, original_speed=15.0,
                               return_threshold_distance=10.0, return_threshold_time=60.0)
        result = plan_return_path(own, cfg, [target])
        assert result is None

    def test_no_targets_always_returns(self):
        own = make_ship(mmsi=123456789, latitude=30.0, longitude=120.0,
                        heading=105.0, sog=15.0)
        cfg = ReturnPathConfig(original_heading=90.0, original_speed=15.0)
        result = plan_return_path(own, cfg, [])
        assert result is not None


class TestDecisionConsistency:

    def test_same_scenario_same_decision(self):
        own = make_ship(mmsi=123456789, latitude=30.0, longitude=120.0,
                        heading=90.0, sog=15.0)
        target = make_ship(mmsi=987654321, latitude=30.0,
                           longitude=120.0 + nm_lon(10.0), heading=270.0, sog=15.0)
        env = EnvironmentConfig()
        situation = analyze_encounter_situation(own, target)
        action = apply_colregs_rule(situation.encounter_type, own, target)
        s1 = plan_avoidance(own, [target], env, action)
        s2 = plan_avoidance(own, [target], env, action)
        assert s1 is not None and s2 is not None
        assert s1.path.path_type == s2.path.path_type

    def test_risk_assessment_deterministic(self):
        own = make_ship(mmsi=123456789, latitude=30.0, longitude=120.0,
                        heading=90.0, sog=15.0)
        target = make_ship(mmsi=987654321, latitude=30.0,
                           longitude=120.0 + nm_lon(10.0), heading=270.0, sog=15.0)
        r1 = assess_collision_risk(own, target)
        r2 = assess_collision_risk(own, target)
        assert abs(r1.cri - r2.cri) < 1e-9
        assert abs(r1.dcpa - r2.dcpa) < 1e-9


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
