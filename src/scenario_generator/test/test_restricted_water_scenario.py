"""
受限水域场景生成器单元测试
Requirements: 2.3
"""
import math
import pytest
from hypothesis import given, settings, strategies as st

from scenario_generator.generator import ScenarioGenerator, RestrictedWaterParams
from scenario_generator.models import ScenarioType, WaterAreaType


@pytest.fixture
def gen():
    return ScenarioGenerator()


class TestRestrictedWaterParams:
    def test_default_valid(self):
        p = RestrictedWaterParams()
        assert p.channel_width_nm == 0.5
        assert p.channel_length_nm == 5.0

    def test_zero_width_invalid(self):
        with pytest.raises(ValueError):
            RestrictedWaterParams(channel_width_nm=0.0)

    def test_too_wide_invalid(self):
        with pytest.raises(ValueError):
            RestrictedWaterParams(channel_width_nm=6.0)

    def test_zero_length_invalid(self):
        with pytest.raises(ValueError):
            RestrictedWaterParams(channel_length_nm=0.0)

    def test_too_few_ships(self):
        with pytest.raises(ValueError):
            RestrictedWaterParams(num_ships=1)

    def test_zero_own_speed(self):
        with pytest.raises(ValueError):
            RestrictedWaterParams(own_speed=0.0)

    def test_zero_target_speed(self):
        with pytest.raises(ValueError):
            RestrictedWaterParams(target_speed=0.0)

    def test_separation_exceeds_length(self):
        with pytest.raises(ValueError):
            RestrictedWaterParams(channel_length_nm=3.0, separation_nm=5.0)

    def test_invalid_latitude(self):
        with pytest.raises(ValueError):
            RestrictedWaterParams(base_latitude=100.0)

    def test_invalid_longitude(self):
        with pytest.raises(ValueError):
            RestrictedWaterParams(base_longitude=200.0)


class TestRestrictedWaterScenario:
    def test_generates_scenario(self, gen):
        p = RestrictedWaterParams()
        sc = gen.generate_restricted_water_scenario(p)
        assert sc is not None

    def test_scenario_type(self, gen):
        p = RestrictedWaterParams()
        sc = gen.generate_restricted_water_scenario(p)
        assert sc.scenario_type == ScenarioType.HEAD_ON

    def test_two_ships_generated(self, gen):
        p = RestrictedWaterParams()
        sc = gen.generate_restricted_water_scenario(p)
        assert len(sc.ships) == 2

    def test_own_ship_at_base_position(self, gen):
        p = RestrictedWaterParams(
            base_latitude=31.0, base_longitude=121.0, use_irregular_channel=False
        )
        sc = gen.generate_restricted_water_scenario(p)
        own = sc.ships[0]
        assert abs(own.latitude - 31.0) < 1e-9
        assert abs(own.longitude - 121.0) < 1e-9

    def test_own_ship_mmsi(self, gen):
        p = RestrictedWaterParams(own_mmsi=111222333)
        sc = gen.generate_restricted_water_scenario(p)
        assert sc.ships[0].mmsi == 111222333

    def test_target_ship_mmsi(self, gen):
        p = RestrictedWaterParams(target_mmsi=444555666)
        sc = gen.generate_restricted_water_scenario(p)
        assert sc.ships[1].mmsi == 444555666

    def test_own_ship_heading_matches_channel(self, gen):
        p = RestrictedWaterParams(channel_heading=45.0, use_irregular_channel=False)
        sc = gen.generate_restricted_water_scenario(p)
        assert abs(sc.ships[0].heading - 45.0) < 1e-9

    def test_target_ship_opposite_heading(self, gen):
        p = RestrictedWaterParams(channel_heading=90.0, use_irregular_channel=False)
        sc = gen.generate_restricted_water_scenario(p)
        expected = (90.0 + 180.0) % 360.0
        assert abs(sc.ships[1].heading - expected) < 1e-9

    def test_target_ahead_of_own(self, gen):
        """目标船应在本船前方（沿航道方向）"""
        p = RestrictedWaterParams(
            channel_heading=90.0,  # 东向
            separation_nm=1.0,
            base_latitude=30.0, base_longitude=120.0,
            use_irregular_channel=False,
        )
        sc = gen.generate_restricted_water_scenario(p)
        own = sc.ships[0]
        target = sc.ships[1]
        # 航向90度（东），目标应在东方，即经度更大
        assert target.longitude > own.longitude

    def test_separation_distance_correct(self, gen):
        """目标船与本船的距离应接近 separation_nm"""
        sep = 1.5
        p = RestrictedWaterParams(
            channel_heading=0.0,  # 北向
            separation_nm=sep,
            base_latitude=30.0, base_longitude=120.0,
            use_irregular_channel=False,
        )
        sc = gen.generate_restricted_water_scenario(p)
        own = sc.ships[0]
        target = sc.ships[1]
        NM = 1 / 60.0
        dlat = (target.latitude - own.latitude) / NM
        dlon = (target.longitude - own.longitude) / NM * math.cos(
            math.radians(own.latitude))
        dist = math.sqrt(dlat**2 + dlon**2)
        assert abs(dist - sep) < 0.1

    def test_environment_restricted_water_type(self, gen):
        p = RestrictedWaterParams()
        sc = gen.generate_restricted_water_scenario(p)
        assert sc.environment is not None
        assert sc.environment.water_area_type == WaterAreaType.RESTRICTED

    def test_map_boundaries_set(self, gen):
        p = RestrictedWaterParams()
        sc = gen.generate_restricted_water_scenario(p)
        assert sc.environment.map_boundaries is not None
        # 弯曲航道为不规则多边形（顶点数 > 4）
        assert len(sc.environment.map_boundaries) >= 4

    def test_irregular_channel_many_vertices(self, gen):
        p = RestrictedWaterParams(use_irregular_channel=True)
        sc = gen.generate_restricted_water_scenario(p)
        assert len(sc.environment.map_boundaries) >= 8

    def test_boundaries_are_tuples(self, gen):
        p = RestrictedWaterParams()
        sc = gen.generate_restricted_water_scenario(p)
        for b in sc.environment.map_boundaries:
            assert len(b) == 2

    def test_custom_scenario_id(self, gen):
        p = RestrictedWaterParams(scenario_id="rw_001")
        sc = gen.generate_restricted_water_scenario(p)
        assert sc.scenario_id == "rw_001"

    def test_description_contains_width(self, gen):
        p = RestrictedWaterParams(channel_width_nm=0.8)
        sc = gen.generate_restricted_water_scenario(p)
        assert "0.8" in sc.description

    def test_duration_set(self, gen):
        p = RestrictedWaterParams(duration=500.0)
        sc = gen.generate_restricted_water_scenario(p)
        assert abs(sc.duration - 500.0) < 1e-9

    def test_own_ship_speed(self, gen):
        p = RestrictedWaterParams(own_speed=6.0)
        sc = gen.generate_restricted_water_scenario(p)
        assert abs(sc.ships[0].sog - 6.0) < 1e-9

    def test_target_ship_speed(self, gen):
        p = RestrictedWaterParams(target_speed=10.0)
        sc = gen.generate_restricted_water_scenario(p)
        assert abs(sc.ships[1].sog - 10.0) < 1e-9


@given(
    width=st.floats(min_value=0.1, max_value=4.9),
    length=st.floats(min_value=0.5, max_value=20.0),
    heading=st.floats(min_value=0.0, max_value=359.9),
)
@settings(max_examples=30)
def test_property_always_two_ships(width, length, heading):
    """Property: 受限水域场景始终生成2艘船"""
    gen = ScenarioGenerator()
    p = RestrictedWaterParams(
        channel_width_nm=width,
        channel_length_nm=length,
        channel_heading=heading,
        separation_nm=min(0.5, length * 0.4),
    )
    sc = gen.generate_restricted_water_scenario(p)
    assert len(sc.ships) == 2


@given(
    width=st.floats(min_value=0.1, max_value=4.9),
    length=st.floats(min_value=1.0, max_value=20.0),
)
@settings(max_examples=30)
def test_property_restricted_water_type(width, length):
    """Property: 环境水域类型始终为RESTRICTED"""
    gen = ScenarioGenerator()
    p = RestrictedWaterParams(
        channel_width_nm=width,
        channel_length_nm=length,
        separation_nm=min(0.5, length * 0.4),
    )
    sc = gen.generate_restricted_water_scenario(p)
    assert sc.environment.water_area_type == WaterAreaType.RESTRICTED


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
