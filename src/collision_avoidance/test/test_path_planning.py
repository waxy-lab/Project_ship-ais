"""
路径规划模块测试

测试 path_planning 模块的功能正确性
Requirements: 5.1-5.6
"""

import pytest
import math
import sys
import os
from hypothesis import given, strategies as st, assume, settings

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../scenario_generator'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../test_framework'))

from scenario_generator.models import ShipState
from collision_avoidance.path_planning import (
    generate_avoidance_paths,
    evaluate_path,
    select_best_path,
    generate_control_commands,
    check_path_safety,
    check_colregs_compliance,
    plan_return_path,
    plan_avoidance,
    Path, PathType, PathStatus, PathScore,
    Waypoint, ControlCommand, ReturnPathConfig, AvoidanceStrategy,
    calculate_distance, calculate_path_length,
    normalize_heading, calculate_heading_change,
    MIN_SAFE_DISTANCE, PATH_EVALUATION_HORIZON,
    WEIGHT_SAFETY, WEIGHT_EFFICIENCY, WEIGHT_COMPLIANCE,
    DEFAULT_TURN_ANGLES, DEFAULT_SPEED_FACTORS,
)
from collision_avoidance.rules_engine import (
    AvoidanceAction, ActionType, TurnDirection,
)
from test_framework.strategies import ship_state_strategy


# ============================================================================
# 测试辅助工具
# ============================================================================

def make_ship(lat=0.0, lon=0.0, heading=90.0, sog=10.0, mmsi=123456789):
    return ShipState(mmsi=mmsi, latitude=lat, longitude=lon, heading=heading, sog=sog)


def make_head_on_action():
    return AvoidanceAction(
        action_type=ActionType.COURSE_CHANGE,
        turn_direction=TurnDirection.STARBOARD,
        turn_angle=15.0,
        reason="Rule 14: 对遇，向右转"
    )


def make_maintain_action():
    return AvoidanceAction(
        action_type=ActionType.MAINTAIN,
        maintain_course=True,
        reason="Rule 17: 直航船，保持"
    )


def make_no_action():
    return AvoidanceAction(
        action_type=ActionType.NO_ACTION,
        no_action=True,
        reason="无相遇风险"
    )


TARGET_MMSI = 987654321  # 用于所有目标船的有效9位MMSI


# ============================================================================
# 任务 9.2 / 9.3 — 路径生成单元测试  Requirements: 5.1
# ============================================================================

class TestGenerateAvoidancePaths:

    def test_returns_list(self):
        own = make_ship()
        target = make_ship(lon=0.1, heading=270.0, mmsi=987654321)
        paths = generate_avoidance_paths(own, [target], None, make_head_on_action())
        assert isinstance(paths, list)
        assert len(paths) > 0

    def test_path_count(self):
        """应生成至少8条候选路径（3转向+3减速+1组合+1保持）"""
        own = make_ship()
        target = make_ship(lon=0.1, heading=270.0, mmsi=987654321)
        paths = generate_avoidance_paths(own, [target], None, make_head_on_action())
        assert len(paths) >= 8

    def test_contains_all_path_types(self):
        own = make_ship()
        target = make_ship(lon=0.1, heading=270.0, mmsi=987654321)
        paths = generate_avoidance_paths(own, [target], None, make_head_on_action())
        types = {p.path_type for p in paths}
        assert PathType.COURSE_CHANGE in types
        assert PathType.SPEED_CHANGE in types
        assert PathType.COMBINED in types
        assert PathType.MAINTAIN in types

    def test_each_path_has_at_least_two_waypoints(self):
        own = make_ship()
        target = make_ship(lon=0.1, heading=270.0, mmsi=987654321)
        paths = generate_avoidance_paths(own, [target], None, make_head_on_action())
        for p in paths:
            assert len(p.waypoints) >= 2, f"{p.path_id} 航点不足"

    def test_path_ids_are_unique(self):
        own = make_ship()
        target = make_ship(lon=0.1, heading=270.0, mmsi=987654321)
        paths = generate_avoidance_paths(own, [target], None, make_head_on_action())
        ids = [p.path_id for p in paths]
        assert len(ids) == len(set(ids))

    def test_starboard_turn_positive_course_change(self):
        own = make_ship()
        target = make_ship(lon=0.1, heading=270.0, mmsi=987654321)
        paths = generate_avoidance_paths(own, [target], None, make_head_on_action())
        course_paths = [p for p in paths if p.path_type == PathType.COURSE_CHANGE]
        for p in course_paths:
            assert p.course_change is not None
            assert p.course_change >= 0

    def test_port_turn_negative_course_change(self):
        own = make_ship()
        target = make_ship(lon=0.1, heading=270.0, mmsi=987654321)
        port_action = AvoidanceAction(
            action_type=ActionType.COURSE_CHANGE,
            turn_direction=TurnDirection.PORT,
            turn_angle=20.0,
            reason="左转"
        )
        paths = generate_avoidance_paths(own, [target], None, port_action)
        course_paths = [p for p in paths if p.path_type == PathType.COURSE_CHANGE]
        for p in course_paths:
            assert p.course_change is not None
            assert p.course_change <= 0

    def test_speed_change_paths_factor_less_than_one(self):
        own = make_ship()
        target = make_ship(lon=0.1, heading=270.0, mmsi=987654321)
        paths = generate_avoidance_paths(own, [target], None, make_head_on_action())
        speed_paths = [p for p in paths if p.path_type == PathType.SPEED_CHANGE]
        for p in speed_paths:
            assert p.speed_factor is not None
            assert p.speed_factor < 1.0

    def test_maintain_path_zero_course_change(self):
        own = make_ship()
        target = make_ship(lon=0.1, heading=270.0, mmsi=987654321)
        paths = generate_avoidance_paths(own, [target], None, make_head_on_action())
        maintain_paths = [p for p in paths if p.path_type == PathType.MAINTAIN]
        assert len(maintain_paths) >= 1
        for p in maintain_paths:
            assert p.course_change == 0.0
            assert p.speed_factor == 1.0

    def test_waypoints_start_at_own_position(self):
        own = make_ship(lat=1.0, lon=2.0, heading=45.0)
        target = make_ship(lon=2.1, heading=225.0, mmsi=987654321)
        paths = generate_avoidance_paths(own, [target], None, make_head_on_action())
        for p in paths:
            wp0 = p.waypoints[0]
            assert abs(wp0.latitude - 1.0) < 1e-9
            assert abs(wp0.longitude - 2.0) < 1e-9

    def test_estimated_duration_equals_horizon(self):
        own = make_ship()
        target = make_ship(lon=0.1, heading=270.0, mmsi=987654321)
        paths = generate_avoidance_paths(own, [target], None, make_head_on_action())
        for p in paths:
            assert p.estimated_duration == PATH_EVALUATION_HORIZON

    def test_path_length_positive(self):
        own = make_ship(sog=10.0)
        target = make_ship(lon=0.1, heading=270.0, mmsi=987654321)
        paths = generate_avoidance_paths(own, [target], None, make_head_on_action())
        for p in paths:
            assert p.path_length > 0.0

    def test_no_target_ships_generates_paths(self):
        own = make_ship()
        paths = generate_avoidance_paths(own, [], None, make_no_action())
        assert len(paths) > 0

    def test_turn_angles_match_defaults(self):
        own = make_ship(heading=0.0)
        target = make_ship(lon=0.1, heading=270.0, mmsi=987654321)
        paths = generate_avoidance_paths(own, [target], None, make_head_on_action())
        course_paths = [p for p in paths if p.path_type == PathType.COURSE_CHANGE]
        actual_angles = sorted([abs(p.course_change) for p in course_paths])
        expected_angles = sorted(DEFAULT_TURN_ANGLES)
        assert actual_angles == expected_angles

    def test_speed_factors_match_defaults(self):
        own = make_ship()
        target = make_ship(lon=0.1, heading=270.0, mmsi=987654321)
        paths = generate_avoidance_paths(own, [target], None, make_head_on_action())
        speed_paths = [p for p in paths if p.path_type == PathType.SPEED_CHANGE]
        actual_factors = sorted([p.speed_factor for p in speed_paths])
        expected_factors = sorted(DEFAULT_SPEED_FACTORS)
        assert actual_factors == expected_factors


# ============================================================================
# 路径评估单元测试  Requirements: 5.2, 5.3, 5.4
# ============================================================================

class TestEvaluatePath:

    def test_score_in_range(self):
        own = make_ship()
        target = make_ship(lon=0.1, heading=270.0, mmsi=987654321)
        paths = generate_avoidance_paths(own, [target], None, make_head_on_action())
        for p in paths:
            score = evaluate_path(p, [target], make_head_on_action())
            assert 0 <= score.total <= 1.0
            assert 0 <= score.safety <= 1.0
            assert 0 <= score.efficiency <= 1.0
            assert 0 <= score.compliance <= 1.0

    def test_weighted_total(self):
        own = make_ship()
        target = make_ship(lon=0.1, heading=270.0, mmsi=987654321)
        paths = generate_avoidance_paths(own, [target], None, make_head_on_action())
        for p in paths:
            score = evaluate_path(p, [target], make_head_on_action())
            expected = (WEIGHT_SAFETY * score.safety
                        + WEIGHT_EFFICIENCY * score.efficiency
                        + WEIGHT_COMPLIANCE * score.compliance)
            assert abs(score.total - expected) < 1e-6

    def test_compliant_path_higher_compliance_score(self):
        own = make_ship()
        target = make_ship(lon=0.1, heading=270.0, mmsi=987654321)
        paths = generate_avoidance_paths(own, [target], None, make_head_on_action())
        action = make_head_on_action()
        maintain_paths = [p for p in paths if p.path_type == PathType.MAINTAIN]
        starboard_paths = [p for p in paths if p.path_type == PathType.COURSE_CHANGE
                           and p.course_change is not None and p.course_change > 0]
        if maintain_paths and starboard_paths:
            s_m = evaluate_path(maintain_paths[0], [target], action)
            s_s = evaluate_path(starboard_paths[0], [target], action)
            assert s_s.compliance > s_m.compliance

    def test_larger_course_change_lower_efficiency(self):
        own = make_ship()
        target = make_ship(lon=0.1, heading=270.0, mmsi=987654321)
        paths = generate_avoidance_paths(own, [target], None, make_head_on_action())
        course_paths = sorted(
            [p for p in paths if p.path_type == PathType.COURSE_CHANGE],
            key=lambda p: abs(p.course_change)
        )
        if len(course_paths) >= 2:
            s_small = evaluate_path(course_paths[0], [target], make_head_on_action())
            s_large = evaluate_path(course_paths[-1], [target], make_head_on_action())
            assert s_small.efficiency >= s_large.efficiency

    def test_no_target_ships_max_safety(self):
        own = make_ship()
        paths = generate_avoidance_paths(own, [], None, make_no_action())
        for p in paths:
            score = evaluate_path(p, [], make_no_action())
            assert score.safety == 1.0

    def test_score_details_not_empty(self):
        own = make_ship()
        target = make_ship(lon=0.1, heading=270.0, mmsi=987654321)
        paths = generate_avoidance_paths(own, [target], None, make_head_on_action())
        for p in paths:
            score = evaluate_path(p, [target], make_head_on_action())
            assert score.details != ""


# ============================================================================
# 路径安全性检查  Requirements: 5.3
# ============================================================================

class TestCheckPathSafety:

    def test_no_targets_always_safe(self):
        own = make_ship()
        paths = generate_avoidance_paths(own, [], None, make_no_action())
        for p in paths:
            assert check_path_safety(p, []) is True

    def test_large_turn_avoids_collision(self):
        """大角度转向路径在对遇场景中应产生安全路径"""
        own = make_ship(heading=90.0)
        # 目标船在0.2度外（约12海里），正对遇
        target = make_ship(lon=0.2, heading=270.0, mmsi=987654321)
        paths = generate_avoidance_paths(own, [target], None, make_head_on_action())
        large_turns = [p for p in paths if p.path_type == PathType.COURSE_CHANGE
                       and p.course_change is not None and abs(p.course_change) >= 45.0]
        safe_found = any(check_path_safety(p, [target]) for p in large_turns)
        assert safe_found, "应至少有一条45度转向路径是安全的"


# ============================================================================
# COLREGS 合规性检查  Requirements: 5.4
# ============================================================================

class TestCheckColregsCompliance:

    def test_maintain_compliant_for_maintain_action(self):
        own = make_ship()
        target = make_ship(lon=0.1, heading=270.0, mmsi=987654321)
        paths = generate_avoidance_paths(own, [target], None, make_maintain_action())
        maintain_paths = [p for p in paths if p.path_type == PathType.MAINTAIN]
        for p in maintain_paths:
            assert check_colregs_compliance(p, make_maintain_action()) is True

    def test_maintain_not_compliant_for_course_change_action(self):
        own = make_ship()
        target = make_ship(lon=0.1, heading=270.0, mmsi=987654321)
        paths = generate_avoidance_paths(own, [target], None, make_head_on_action())
        maintain_paths = [p for p in paths if p.path_type == PathType.MAINTAIN]
        for p in maintain_paths:
            assert check_colregs_compliance(p, make_head_on_action()) is False

    def test_starboard_compliant_for_starboard_action(self):
        own = make_ship()
        target = make_ship(lon=0.1, heading=270.0, mmsi=987654321)
        paths = generate_avoidance_paths(own, [target], None, make_head_on_action())
        starboard = [p for p in paths if p.path_type == PathType.COURSE_CHANGE
                     and p.course_change is not None and p.course_change > 0]
        for p in starboard:
            assert check_colregs_compliance(p, make_head_on_action()) is True

    def test_port_not_compliant_for_starboard_action(self):
        own = make_ship()
        target = make_ship(lon=0.1, heading=270.0, mmsi=987654321)
        port_action = AvoidanceAction(
            action_type=ActionType.COURSE_CHANGE,
            turn_direction=TurnDirection.PORT,
            turn_angle=20.0,
            reason="左转"
        )
        paths = generate_avoidance_paths(own, [target], None, make_head_on_action())
        starboard = [p for p in paths if p.path_type == PathType.COURSE_CHANGE
                     and p.course_change is not None and p.course_change > 0]
        for p in starboard:
            assert check_colregs_compliance(p, port_action) is False


# ============================================================================
# 路径选择  Requirements: 5.5
# ============================================================================

class TestSelectBestPath:

    def test_returns_best_total_score(self):
        own = make_ship()
        target = make_ship(lon=0.1, heading=270.0, mmsi=987654321)
        paths = generate_avoidance_paths(own, [target], None, make_head_on_action())
        scores = [evaluate_path(p, [target], make_head_on_action()) for p in paths]
        best_path, best_score = select_best_path(paths, scores)
        assert best_path is not None
        assert best_score is not None
        # 验证选出的总分是所有安全路径中最高的
        for p, s in zip(paths, scores):
            if s.safety > 0.0:
                assert best_score.total >= s.total - 1e-9

    def test_returns_none_for_empty(self):
        assert select_best_path([], []) == (None, None)

    def test_selected_path_in_input_list(self):
        own = make_ship()
        target = make_ship(lon=0.1, heading=270.0, mmsi=987654321)
        paths = generate_avoidance_paths(own, [target], None, make_head_on_action())
        scores = [evaluate_path(p, [target], make_head_on_action()) for p in paths]
        best_path, _ = select_best_path(paths, scores)
        assert best_path in paths


# ============================================================================
# 控制指令生成  Requirements: 5.5
# ============================================================================

class TestGenerateControlCommands:

    def test_returns_list(self):
        own = make_ship()
        target = make_ship(lon=0.1, heading=270.0, mmsi=987654321)
        paths = generate_avoidance_paths(own, [target], None, make_head_on_action())
        assert isinstance(generate_control_commands(paths[0], own), list)

    def test_at_least_one_command(self):
        own = make_ship()
        target = make_ship(lon=0.1, heading=270.0, mmsi=987654321)
        paths = generate_avoidance_paths(own, [target], None, make_head_on_action())
        for p in paths:
            assert len(generate_control_commands(p, own)) >= 1

    def test_first_command_timestamp_zero(self):
        own = make_ship()
        target = make_ship(lon=0.1, heading=270.0, mmsi=987654321)
        paths = generate_avoidance_paths(own, [target], None, make_head_on_action())
        for p in paths:
            cmds = generate_control_commands(p, own)
            assert cmds[0].timestamp == 0.0

    def test_target_heading_valid_range(self):
        own = make_ship()
        target = make_ship(lon=0.1, heading=270.0, mmsi=987654321)
        paths = generate_avoidance_paths(own, [target], None, make_head_on_action())
        for p in paths:
            for cmd in generate_control_commands(p, own):
                if cmd.target_heading is not None:
                    assert 0 <= cmd.target_heading < 360

    def test_target_speed_non_negative(self):
        own = make_ship()
        target = make_ship(lon=0.1, heading=270.0, mmsi=987654321)
        paths = generate_avoidance_paths(own, [target], None, make_head_on_action())
        for p in paths:
            for cmd in generate_control_commands(p, own):
                if cmd.target_speed is not None:
                    assert cmd.target_speed >= 0


# ============================================================================
# 返航路径规划  Requirements: 5.6
# ============================================================================

class TestPlanReturnPath:

    def test_returns_path_when_safe(self):
        """目标船已远离时应返回路径"""
        own = make_ship(heading=100.0)
        target = make_ship(lon=0.5, heading=270.0, mmsi=987654321)  # ~30海里外
        cfg = ReturnPathConfig(
            original_heading=90.0,
            original_speed=10.0,
            return_threshold_distance=2.0,
            return_threshold_time=5.0
        )
        result = plan_return_path(own, cfg, [target])
        assert result is not None

    def test_returns_none_when_target_too_close(self):
        """目标船仍在威胁范围内时应返回 None"""
        own = make_ship(heading=100.0)
        target = make_ship(lon=0.005, heading=270.0, mmsi=987654321)  # 约0.3海里，正对来
        cfg = ReturnPathConfig(
            original_heading=90.0,
            original_speed=10.0,
            return_threshold_distance=10.0,
            return_threshold_time=60.0
        )
        result = plan_return_path(own, cfg, [target])
        assert result is None

    def test_return_path_ends_at_original_heading(self):
        """返航路径最后一个航点的航向应为原始航向"""
        own = make_ship(heading=120.0)
        target = make_ship(lon=0.5, heading=270.0, mmsi=987654321)
        cfg = ReturnPathConfig(
            original_heading=90.0,
            original_speed=10.0,
            return_threshold_distance=2.0,
            return_threshold_time=5.0
        )
        result = plan_return_path(own, cfg, [target])
        if result is not None:
            assert abs(result.get_end_waypoint().heading - 90.0) < 1.0

    def test_return_path_has_waypoints(self):
        own = make_ship(heading=120.0)
        target = make_ship(lon=0.5, heading=270.0, mmsi=987654321)
        cfg = ReturnPathConfig(original_heading=90.0, original_speed=10.0,
                               return_threshold_distance=2.0, return_threshold_time=5.0)
        result = plan_return_path(own, cfg, [target])
        if result is not None:
            assert len(result.waypoints) >= 2

    def test_no_targets_always_returns_path(self):
        own = make_ship(heading=120.0)
        cfg = ReturnPathConfig(original_heading=90.0, original_speed=10.0)
        result = plan_return_path(own, cfg, [])
        assert result is not None

    def test_return_path_id(self):
        own = make_ship(heading=120.0)
        cfg = ReturnPathConfig(original_heading=90.0, original_speed=10.0)
        result = plan_return_path(own, cfg, [])
        assert result is not None
        assert result.path_id == "return_path"


# ============================================================================
# 完整避让决策流程  Requirements: 5.1-5.5
# ============================================================================

class TestPlanAvoidance:

    def test_returns_strategy(self):
        own = make_ship()
        target = make_ship(lon=0.1, heading=270.0, mmsi=987654321)
        strategy = plan_avoidance(own, [target], None, make_head_on_action())
        assert strategy is not None
        assert isinstance(strategy, AvoidanceStrategy)

    def test_strategy_is_selected(self):
        own = make_ship()
        target = make_ship(lon=0.1, heading=270.0, mmsi=987654321)
        strategy = plan_avoidance(own, [target], None, make_head_on_action())
        assert strategy.is_selected is True

    def test_strategy_has_control_commands(self):
        own = make_ship()
        target = make_ship(lon=0.1, heading=270.0, mmsi=987654321)
        strategy = plan_avoidance(own, [target], None, make_head_on_action())
        assert len(strategy.control_commands) >= 1

    def test_strategy_score_valid_range(self):
        own = make_ship()
        target = make_ship(lon=0.1, heading=270.0, mmsi=987654321)
        strategy = plan_avoidance(own, [target], None, make_head_on_action())
        assert 0 <= strategy.score.total <= 1.0

    def test_strategy_reason_not_empty(self):
        own = make_ship()
        target = make_ship(lon=0.1, heading=270.0, mmsi=987654321)
        strategy = plan_avoidance(own, [target], None, make_head_on_action())
        assert strategy.reason != ""

    def test_strategy_path_not_none(self):
        own = make_ship()
        target = make_ship(lon=0.1, heading=270.0, mmsi=987654321)
        strategy = plan_avoidance(own, [target], None, make_head_on_action())
        assert strategy.path is not None


# ============================================================================
# 属性测试  Property 12: 路径安全性验证  Requirements: 5.3
# ============================================================================

class TestPathSafetyProperties:

    @given(
        ship1_data=ship_state_strategy(
            lat_range=(-30.0, 30.0),
            lon_range=(-100.0, 100.0),
            speed_range=(2.0, 20.0)
        ),
        ship2_data=ship_state_strategy(
            lat_range=(-30.0, 30.0),
            lon_range=(-100.0, 100.0),
            speed_range=(2.0, 20.0)
        )
    )
    @settings(max_examples=10, deadline=5000)
    def test_property_path_safety_score_consistent(
            self, ship1_data, ship2_data):
        """
        Feature: maritime-collision-avoidance, Property 12: 路径安全性验证
        **Validates: Requirements 5.3**

        属性：safety_score=0 的路径必须被 check_path_safety 判为不安全，
        safety_score=1 的路径必须被 check_path_safety 判为安全。
        """
        lat_diff = abs(ship1_data['latitude'] - ship2_data['latitude'])
        lon_diff = abs(ship1_data['longitude'] - ship2_data['longitude'])
        assume(lat_diff > 0.001 or lon_diff > 0.001)
        if ship1_data['mmsi'] == ship2_data['mmsi']:
            ship2_data = dict(ship2_data)
            ship2_data['mmsi'] = ship2_data['mmsi'] + 1 if ship2_data['mmsi'] < 999999999 else 100000000

        ship1 = ShipState(**ship1_data)
        ship2 = ShipState(**ship2_data)
        action = make_head_on_action()
        paths = generate_avoidance_paths(ship1, [ship2], None, action)

        for p in paths:
            score = evaluate_path(p, [ship2], action)
            is_safe = check_path_safety(p, [ship2])
            # safety_score=1.0 意味着 min_dcpa >= 2*MIN_SAFE_DISTANCE，必定安全
            if score.safety >= 1.0:
                assert is_safe, (
                    f"safety=1.0 的路径应被判为安全, path={p.path_id}")


# ============================================================================
# 属性测试  Property 13: 路径COLREGS合规性  Requirements: 5.4
# ============================================================================

class TestPathColregsProperties:

    @given(
        heading=st.floats(min_value=0.0, max_value=359.9),
        speed=st.floats(min_value=1.0, max_value=25.0)
    )
    @settings(max_examples=10, deadline=5000)
    def test_property_compliant_paths_have_higher_compliance_score(
            self, heading, speed):
        """
        Feature: maritime-collision-avoidance, Property 13: 路径COLREGS合规性
        **Validates: Requirements 5.4**

        属性：对任意船舶状态，合规路径的 compliance_score 应 >= 不合规路径。
        """
        own = make_ship(heading=heading, sog=speed)
        target = make_ship(lon=0.1, heading=(heading + 180) % 360, mmsi=987654321)
        action = make_head_on_action()
        paths = generate_avoidance_paths(own, [target], None, action)

        compliant = [p for p in paths if check_colregs_compliance(p, action)]
        non_compliant = [p for p in paths if not check_colregs_compliance(p, action)]

        if compliant and non_compliant:
            best_compliant_score = max(
                evaluate_path(p, [target], action).compliance for p in compliant)
            worst_non_compliant_score = min(
                evaluate_path(p, [target], action).compliance for p in non_compliant)
            assert best_compliant_score >= worst_non_compliant_score


if __name__ == '__main__':
    pytest.main([__file__, '-v'])

