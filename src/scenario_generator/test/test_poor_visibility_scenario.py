"""
能见度不良场景生成器单元测试
Requirements: 2.5
"""
import pytest
from hypothesis import given, settings, strategies as st

from scenario_generator.generator import ScenarioGenerator, PoorVisibilityParams
from scenario_generator.models import ScenarioType, Visibility


@pytest.fixture
def gen():
    return ScenarioGenerator()


class TestPoorVisibilityParams:
    def test_default_valid(self):
        p = PoorVisibilityParams()
        assert p.visibility_nm == 1.0
        assert p.risk_threshold_warning == 0.3
        assert p.risk_threshold_danger == 0.5

    def test_zero_visibility_invalid(self):
        with pytest.raises(ValueError):
            PoorVisibilityParams(visibility_nm=0.0)

    def test_visibility_too_large(self):
        with pytest.raises(ValueError):
            PoorVisibilityParams(visibility_nm=11.0)

    def test_zero_distance_invalid(self):
        with pytest.raises(ValueError):
            PoorVisibilityParams(distance_nm=0.0)

    def test_distance_too_large(self):
        with pytest.raises(ValueError):
            PoorVisibilityParams(distance_nm=25.0)

    def test_zero_own_speed(self):
        with pytest.raises(ValueError):
            PoorVisibilityParams(own_speed=0.0)

    def test_zero_target_speed(self):
        with pytest.raises(ValueError):
            PoorVisibilityParams(target_speed=0.0)

    def test_warning_threshold_out_of_range(self):
        with pytest.raises(ValueError):
            PoorVisibilityParams(risk_threshold_warning=0.0)

    def test_danger_threshold_out_of_range(self):
        with pytest.raises(ValueError):
            PoorVisibilityParams(risk_threshold_danger=1.1)

    def test_warning_ge_danger_invalid(self):
        with pytest.raises(ValueError):
            PoorVisibilityParams(
                risk_threshold_warning=0.6,
                risk_threshold_danger=0.5
            )

    def test_zero_safe_distance_invalid(self):
        with pytest.raises(ValueError):
            PoorVisibilityParams(safe_distance_nm=0.0)

    def test_zero_safe_time_invalid(self):
        with pytest.raises(ValueError):
            PoorVisibilityParams(safe_time_min=0.0)

    def test_invalid_latitude(self):
        with pytest.raises(ValueError):
            PoorVisibilityParams(base_latitude=95.0)

    def test_invalid_longitude(self):
        with pytest.raises(ValueError):
            PoorVisibilityParams(base_longitude=200.0)

    def test_conservative_thresholds_lower_than_normal(self):
        """能见度不良时阈值应低于正常值（0.5/0.7）"""
        p = PoorVisibilityParams()
        assert p.risk_threshold_warning < 0.5
        assert p.risk_threshold_danger < 0.7

    def test_conservative_safe_distance_larger_than_normal(self):
        """能见度不良时安全距离应大于正常值（2nm）"""
        p = PoorVisibilityParams()
        assert p.safe_distance_nm > 2.0

    def test_conservative_safe_time_larger_than_normal(self):
        """能见度不良时安全时间应大于正常值（10min）"""
        p = PoorVisibilityParams()
        assert p.safe_time_min > 10.0


class TestPoorVisibilityScenario:
    def test_generates_scenario(self, gen):
        p = PoorVisibilityParams()
        sc = gen.generate_poor_visibility_scenario(p)
        assert sc is not None

    def test_two_ships(self, gen):
        p = PoorVisibilityParams()
        sc = gen.generate_poor_visibility_scenario(p)
        assert len(sc.ships) == 2

    def test_scenario_type_head_on(self, gen):
        p = PoorVisibilityParams()
        sc = gen.generate_poor_visibility_scenario(p)
        assert sc.scenario_type == ScenarioType.HEAD_ON

    def test_own_ship_at_base_position(self, gen):
        p = PoorVisibilityParams(base_latitude=31.0, base_longitude=121.0)
        sc = gen.generate_poor_visibility_scenario(p)
        own = sc.ships[0]
        assert abs(own.latitude - 31.0) < 1e-9
        assert abs(own.longitude - 121.0) < 1e-9

    def test_own_ship_mmsi(self, gen):
        p = PoorVisibilityParams(own_mmsi=111222333)
        sc = gen.generate_poor_visibility_scenario(p)
        assert sc.ships[0].mmsi == 111222333

    def test_target_ship_mmsi(self, gen):
        p = PoorVisibilityParams(target_mmsi=444555666)
        sc = gen.generate_poor_visibility_scenario(p)
        assert sc.ships[1].mmsi == 444555666

    def test_own_ship_heading(self, gen):
        p = PoorVisibilityParams(own_heading=45.0)
        sc = gen.generate_poor_visibility_scenario(p)
        assert abs(sc.ships[0].heading - 45.0) < 1e-9

    def test_target_opposite_heading(self, gen):
        p = PoorVisibilityParams(own_heading=90.0)
        sc = gen.generate_poor_visibility_scenario(p)
        assert abs(sc.ships[1].heading - 270.0) < 1e-9

    def test_own_ship_speed(self, gen):
        p = PoorVisibilityParams(own_speed=6.0)
        sc = gen.generate_poor_visibility_scenario(p)
        assert abs(sc.ships[0].sog - 6.0) < 1e-9

    def test_target_ship_speed(self, gen):
        p = PoorVisibilityParams(target_speed=7.0)
        sc = gen.generate_poor_visibility_scenario(p)
        assert abs(sc.ships[1].sog - 7.0) < 1e-9

    def test_poor_visibility_enum(self, gen):
        p = PoorVisibilityParams(visibility_nm=1.0)
        sc = gen.generate_poor_visibility_scenario(p)
        assert sc.environment.visibility == Visibility.POOR

    def test_moderate_visibility_enum(self, gen):
        p = PoorVisibilityParams(visibility_nm=3.0)
        sc = gen.generate_poor_visibility_scenario(p)
        assert sc.environment.visibility == Visibility.MODERATE

    def test_good_visibility_enum(self, gen):
        p = PoorVisibilityParams(visibility_nm=6.0)
        sc = gen.generate_poor_visibility_scenario(p)
        assert sc.environment.visibility == Visibility.GOOD

    def test_description_contains_visibility(self, gen):
        p = PoorVisibilityParams(visibility_nm=1.5)
        sc = gen.generate_poor_visibility_scenario(p)
        assert '1.5' in sc.description

    def test_description_contains_thresholds(self, gen):
        p = PoorVisibilityParams(
            risk_threshold_warning=0.3,
            risk_threshold_danger=0.5
        )
        sc = gen.generate_poor_visibility_scenario(p)
        assert '0.30' in sc.description
        assert '0.50' in sc.description

    def test_duration_set(self, gen):
        p = PoorVisibilityParams(duration=700.0)
        sc = gen.generate_poor_visibility_scenario(p)
        assert abs(sc.duration - 700.0) < 1e-9

    def test_custom_scenario_id(self, gen):
        p = PoorVisibilityParams(scenario_id='pv_001')
        sc = gen.generate_poor_visibility_scenario(p)
        assert sc.scenario_id == 'pv_001'

    def test_target_ahead_of_own_north(self, gen):
        p = PoorVisibilityParams(
            own_heading=0.0,
            distance_nm=3.0,
            base_latitude=30.0
        )
        sc = gen.generate_poor_visibility_scenario(p)
        assert sc.ships[1].latitude > sc.ships[0].latitude

    def test_safe_distance_in_description(self, gen):
        p = PoorVisibilityParams(safe_distance_nm=3.5)
        sc = gen.generate_poor_visibility_scenario(p)
        assert '3.5' in sc.description


@given(
    vis=st.floats(min_value=0.1, max_value=9.9,
                  allow_nan=False, allow_infinity=False),
    dist=st.floats(min_value=0.5, max_value=15.0,
                   allow_nan=False, allow_infinity=False),
)
@settings(max_examples=30)
def test_property_always_two_ships(vis, dist):
    """Property: 能见度不良场景始终生成2艘船"""
    gen = ScenarioGenerator()
    p = PoorVisibilityParams(visibility_nm=vis, distance_nm=dist)
    sc = gen.generate_poor_visibility_scenario(p)
    assert len(sc.ships) == 2


@given(
    vis=st.floats(min_value=0.1, max_value=1.99,
                  allow_nan=False, allow_infinity=False),
)
@settings(max_examples=20)
def test_property_low_vis_is_poor_enum(vis):
    """Property: 能见度<2nm始终映射为POOR"""
    gen = ScenarioGenerator()
    p = PoorVisibilityParams(visibility_nm=vis)
    sc = gen.generate_poor_visibility_scenario(p)
    assert sc.environment.visibility == Visibility.POOR


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
