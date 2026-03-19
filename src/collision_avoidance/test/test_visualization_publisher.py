"""
可视化发布节点逻辑单元测试
不依赖 ROS2，测试纯逻辑部分
Requirements: 7.4
"""
import json
import math
import time
import pytest
from hypothesis import given, settings, strategies as st


# ============================================================================
# 复制节点中的纯逻辑函数（不依赖ROS2）
# ============================================================================

RISK_LEVELS = [
    (0.7, 'danger'),
    (0.3, 'warning'),
    (0.0, 'safe'),
]


def get_risk_level(cri: float) -> str:
    for threshold, level in RISK_LEVELS:
        if cri >= threshold:
            return level
    return 'safe'


def build_risk_entry(target_mmsi, dcpa, tcpa, cri, distance, now=0.0):
    level = get_risk_level(cri)
    return {
        'target_mmsi': target_mmsi,
        'dcpa_nm': round(dcpa, 4),
        'tcpa_min': round(tcpa, 2),
        'cri': round(cri, 4),
        'risk_level': level,
        'distance_nm': round(distance, 4),
        'timestamp': now,
    }


def build_risk_payload(own_mmsi, own_hdg, own_sog, own_lat, own_lon,
                       risk_list, now=0.0):
    return {
        'own_mmsi': own_mmsi,
        'own_heading': round(own_hdg, 1),
        'own_sog': round(own_sog, 1),
        'own_lat': round(own_lat, 6),
        'own_lon': round(own_lon, 6),
        'target_risks': sorted(risk_list, key=lambda x: x['cri'], reverse=True),
        'timestamp': now,
    }


def build_summary(total_ships, own_mmsi, target_count, risk_list, now=0.0):
    return {
        'total_ships': total_ships,
        'own_mmsi': own_mmsi,
        'target_count': target_count,
        'high_risk_count': len([r for r in risk_list
                                if r['risk_level'] == 'danger']),
        'warning_count': len([r for r in risk_list
                              if r['risk_level'] == 'warning']),
        'timestamp': now,
    }


# ============================================================================
# 风险等级映射测试
# ============================================================================

class TestRiskLevel:
    def test_danger_above_07(self):
        assert get_risk_level(0.7) == 'danger'
        assert get_risk_level(0.9) == 'danger'
        assert get_risk_level(1.0) == 'danger'

    def test_warning_between_03_07(self):
        assert get_risk_level(0.3) == 'warning'
        assert get_risk_level(0.5) == 'warning'
        assert get_risk_level(0.699) == 'warning'

    def test_safe_below_03(self):
        assert get_risk_level(0.0) == 'safe'
        assert get_risk_level(0.1) == 'safe'
        assert get_risk_level(0.299) == 'safe'

    def test_boundary_03(self):
        assert get_risk_level(0.3) == 'warning'

    def test_boundary_07(self):
        assert get_risk_level(0.7) == 'danger'


# ============================================================================
# 风险条目构建测试
# ============================================================================

class TestRiskEntry:
    def test_has_all_fields(self):
        entry = build_risk_entry(222222222, 0.5, 10.0, 0.4, 1.2)
        for f in ['target_mmsi', 'dcpa_nm', 'tcpa_min', 'cri',
                  'risk_level', 'distance_nm', 'timestamp']:
            assert f in entry

    def test_mmsi_correct(self):
        entry = build_risk_entry(999, 0.5, 10.0, 0.4, 1.2)
        assert entry['target_mmsi'] == 999

    def test_cri_rounded(self):
        entry = build_risk_entry(111, 0.5, 10.0, 0.123456789, 1.0)
        assert entry['cri'] == round(0.123456789, 4)

    def test_risk_level_danger(self):
        entry = build_risk_entry(111, 0.1, 5.0, 0.8, 0.5)
        assert entry['risk_level'] == 'danger'

    def test_risk_level_warning(self):
        entry = build_risk_entry(111, 0.5, 10.0, 0.5, 1.0)
        assert entry['risk_level'] == 'warning'

    def test_risk_level_safe(self):
        entry = build_risk_entry(111, 2.0, 30.0, 0.1, 3.0)
        assert entry['risk_level'] == 'safe'


# ============================================================================
# 风险载荷构建测试
# ============================================================================

class TestRiskPayload:
    def test_has_required_fields(self):
        payload = build_risk_payload(111111111, 90.0, 12.0, 30.0, 120.0, [])
        for f in ['own_mmsi', 'own_heading', 'own_sog', 'own_lat',
                  'own_lon', 'target_risks', 'timestamp']:
            assert f in payload

    def test_sorted_by_cri_descending(self):
        r1 = build_risk_entry(111, 0.5, 10.0, 0.2, 1.0)
        r2 = build_risk_entry(222, 0.2, 5.0, 0.8, 0.5)
        r3 = build_risk_entry(333, 0.8, 15.0, 0.5, 2.0)
        payload = build_risk_payload(999999999, 0.0, 10.0, 30.0, 120.0,
                                     [r1, r2, r3])
        cris = [r['cri'] for r in payload['target_risks']]
        assert cris == sorted(cris, reverse=True)

    def test_json_serializable(self):
        r = build_risk_entry(222222222, 0.5, 10.0, 0.4, 1.2)
        payload = build_risk_payload(111111111, 90.0, 12.0, 30.0, 120.0, [r])
        # 应能正常序列化为JSON
        s = json.dumps(payload)
        parsed = json.loads(s)
        assert parsed['own_mmsi'] == 111111111

    def test_empty_targets(self):
        payload = build_risk_payload(111111111, 0.0, 10.0, 30.0, 120.0, [])
        assert payload['target_risks'] == []


# ============================================================================
# 摘要构建测试
# ============================================================================

class TestSummary:
    def test_has_required_fields(self):
        s = build_summary(3, 111111111, 2, [])
        for f in ['total_ships', 'own_mmsi', 'target_count',
                  'high_risk_count', 'warning_count', 'timestamp']:
            assert f in s

    def test_high_risk_count(self):
        risks = [
            build_risk_entry(111, 0.1, 2.0, 0.8, 0.3),  # danger
            build_risk_entry(222, 0.3, 5.0, 0.9, 0.5),  # danger
            build_risk_entry(333, 0.5, 10.0, 0.4, 1.0),  # warning
        ]
        s = build_summary(4, 999999999, 3, risks)
        assert s['high_risk_count'] == 2
        assert s['warning_count'] == 1

    def test_all_safe(self):
        risks = [build_risk_entry(111, 2.0, 30.0, 0.1, 3.0)]
        s = build_summary(2, 999999999, 1, risks)
        assert s['high_risk_count'] == 0
        assert s['warning_count'] == 0

    def test_json_serializable(self):
        s = build_summary(3, 111111111, 2,
                          [build_risk_entry(222222222, 0.5, 10.0, 0.5, 1.0)])
        parsed = json.loads(json.dumps(s))
        assert parsed['total_ships'] == 3


# ============================================================================
# 属性测试
# ============================================================================

@given(cri=st.floats(min_value=0.0, max_value=1.0,
                     allow_nan=False, allow_infinity=False))
@settings(max_examples=50)
def test_property_risk_level_coverage(cri):
    """Property: 任意CRI值都能得到合法的风险等级"""
    level = get_risk_level(cri)
    assert level in ('safe', 'warning', 'danger')


@given(cri=st.floats(min_value=0.7, max_value=1.0,
                     allow_nan=False, allow_infinity=False))
@settings(max_examples=30)
def test_property_high_cri_is_danger(cri):
    """Property: CRI >= 0.7 始终为 danger"""
    assert get_risk_level(cri) == 'danger'


@given(
    mmsi=st.integers(min_value=100000000, max_value=999999999),
    dcpa=st.floats(min_value=0.0, max_value=20.0,
                   allow_nan=False, allow_infinity=False),
    cri=st.floats(min_value=0.0, max_value=1.0,
                  allow_nan=False, allow_infinity=False),
)
@settings(max_examples=30)
def test_property_entry_json_serializable(mmsi, dcpa, cri):
    """Property: 风险条目始终可JSON序列化"""
    entry = build_risk_entry(mmsi, dcpa, 10.0, cri, dcpa + 0.1)
    s = json.dumps(entry)
    parsed = json.loads(s)
    assert parsed['target_mmsi'] == mmsi


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
