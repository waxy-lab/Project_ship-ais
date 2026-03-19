"""
恶劣天气场景生成器单元测试
Requirements: 2.4
"""
import math
import pytest
from hypothesis import given, settings, strategies as st

from scenario_generator.generator import ScenarioGenerator, RoughWeatherParams
from scenario_generator.models import (
    ScenarioType, WeatherCondition, Visibility
)


@pytest.fixture
def gen():
    return ScenarioGenerator()


class TestRoughWeatherParams:
    def test_default_valid(self):
        p = RoughWeatherParams()
        assert p.wind_speed_ms == 15.0
        assert p.weather_condition == 'rough'

    def test_negative_wind_speed(self):
        with pytest.raises(ValueError):
            RoughWeatherParams(wind_speed_ms=-1.0)

    def test_wind_speed_too_high(self):
        with pytest.raises(ValueError):
            RoughWeatherParams(wind_speed_ms=55.0)

    def test_invalid_wind_direction(self):
        with pytest.raises(ValueError):
            RoughWeatherParams(wind_direction=360.0)

    def test_negative_current_speed(self):
        with pytest.raises(ValueError):
            RoughWeatherParams(current_speed_ms=-0.1)

    def test_current_speed_too_high(self):
        with pytest.raises(ValueError):
            RoughWeatherParams(current_speed_ms=6.0)

    def test_invalid_current_direction(self):
        with pytest.raises(ValueError):
            RoughWeatherParams(current_direction=-1.0)

    def test_zero_visibility(self):
        with pytest.raises(ValueError):
            RoughWeatherParams(visibility_nm=0.0)

    def test_zero_distance(self):
        with pytest.raises(ValueError):
            RoughWeatherParams(distance_nm=0.0)

    def test_distance_too_large(self):
        with pytest.raises(ValueError):
            RoughWeatherParams(distance_nm=25.0)

    def test_zero_own_speed(self):
        with pytest.raises(ValueError):
            RoughWeatherParams(own_speed=0.0)

    def test_zero_target_speed(self):
        with pytest.raises(ValueError):
            RoughWeatherParams(target_speed=0.0)

    def test_invalid_latitude(self):
        with pytest.raises(ValueError):
            RoughWeatherParams(base_latitude=91.0)

    def test_invalid_longitude(self):
        with pytest.raises(ValueError):
            RoughWeatherParams(base_longitude=-181.0)


class TestRoughWeatherScenario:
    def test_generates_scenario(self, gen):
        p = RoughWeatherParams()
        sc = gen.generate_rough_weather_scenario(p)
        assert sc is not None

    def test_two_ships_generated(self, gen):
        p = RoughWeatherParams()
        sc = gen.generate_rough_weather_scenario(p)
        assert len(sc.ships) == 2

    def test_own_ship_at_base_position(self, gen):
        p = RoughWeatherParams(base_latitude=31.0, base_longitude=121.0)
        sc = gen.generate_rough_weather_scenario(p)
        own = sc.ships[0]
        assert abs(own.latitude - 31.0) < 1e-9
        assert abs(own.longitude - 121.0) < 1e-9

    def test_own_ship_mmsi(self, gen):
        p = RoughWeatherParams(own_mmsi=111222333)
        sc = gen.generate_rough_weather_scenario(p)
        assert sc.ships[0].mmsi == 111222333

    def test_target_ship_mmsi(self, gen):
        p = RoughWeatherParams(target_mmsi=444555666)
        sc = gen.generate_rough_weather_scenario(p)
        assert sc.ships[1].mmsi == 444555666

    def test_own_ship_heading(self, gen):
        p = RoughWeatherParams(own_heading=45.0)
        sc = gen.generate_rough_weather_scenario(p)
        assert abs(sc.ships[0].heading - 45.0) < 1e-9

    def test_target_opposite_heading(self, gen):
        p = RoughWeatherParams(own_heading=90.0)
        sc = gen.generate_rough_weather_scenario(p)
        assert abs(sc.ships[1].heading - 270.0) < 1e-9

    def test_own_ship_speed(self, gen):
        p = RoughWeatherParams(own_speed=7.0)
        sc = gen.generate_rough_weather_scenario(p)
        assert abs(sc.ships[0].sog - 7.0) < 1e-9

    def test_target_ship_speed(self, gen):
        p = RoughWeatherParams(target_speed=6.0)
        sc = gen.generate_rough_weather_scenario(p)
        assert abs(sc.ships[1].sog - 6.0) < 1e-9

    def test_rough_weather_condition(self, gen):
        p = RoughWeatherParams(weather_condition='rough')
        sc = gen.generate_rough_weather_scenario(p)
        assert sc.environment.weather_condition == WeatherCondition.ROUGH

    def test_moderate_weather_condition(self, gen):
        p = RoughWeatherParams(weather_condition='moderate')
        sc = gen.generate_rough_weather_scenario(p)
        assert sc.environment.weather_condition == WeatherCondition.MODERATE

    def test_calm_weather_condition(self, gen):
        p = RoughWeatherParams(weather_condition='calm')
        sc = gen.generate_rough_weather_scenario(p)
        assert sc.environment.weather_condition == WeatherCondition.CALM

    def test_wind_speed_set(self, gen):
        p = RoughWeatherParams(wind_speed_ms=20.0)
        sc = gen.generate_rough_weather_scenario(p)
        assert abs(sc.environment.wind_speed - 20.0) < 1e-9

    def test_wind_direction_set(self, gen):
        p = RoughWeatherParams(wind_direction=270.0)
        sc = gen.generate_rough_weather_scenario(p)
        assert abs(sc.environment.wind_direction - 270.0) < 1e-9

    def test_current_speed_set(self, gen):
        p = RoughWeatherParams(current_speed_ms=2.0)
        sc = gen.generate_rough_weather_scenario(p)
        assert abs(sc.environment.current_speed - 2.0) < 1e-9

    def test_current_direction_set(self, gen):
        p = RoughWeatherParams(current_direction=180.0)
        sc = gen.generate_rough_weather_scenario(p)
        assert abs(sc.environment.current_direction - 180.0) < 1e-9

    def test_good_visibility(self, gen):
        p = RoughWeatherParams(visibility_nm=6.0)
        sc = gen.generate_rough_weather_scenario(p)
        assert sc.environment.visibility == Visibility.GOOD

    def test_moderate_visibility(self, gen):
        p = RoughWeatherParams(visibility_nm=3.0)
        sc = gen.generate_rough_weather_scenario(p)
        assert sc.environment.visibility == Visibility.MODERATE

    def test_poor_visibility(self, gen):
        p = RoughWeatherParams(visibility_nm=1.0)
        sc = gen.generate_rough_weather_scenario(p)
        assert sc.environment.visibility == Visibility.POOR

    def test_description_contains_wind_speed(self, gen):
        p = RoughWeatherParams(wind_speed_ms=18.0)
        sc = gen.generate_rough_weather_scenario(p)
        assert '18.0' in sc.description

    def test_duration_set(self, gen):
        p = RoughWeatherParams(duration=800.0)
        sc = gen.generate_rough_weather_scenario(p)
        assert abs(sc.duration - 800.0) < 1e-9

    def test_custom_scenario_id(self, gen):
        p = RoughWeatherParams(scenario_id='rw_weather_001')
        sc = gen.generate_rough_weather_scenario(p)
        assert sc.scenario_id == 'rw_weather_001'

    def test_target_ahead_of_own(self, gen):
        p = RoughWeatherParams(
            own_heading=0.0,  # 北向
            distance_nm=3.0,
            base_latitude=30.0, base_longitude=120.0
        )
        sc = gen.generate_rough_weather_scenario(p)
        assert sc.ships[1].latitude > sc.ships[0].latitude


@given(
    wind=st.floats(min_value=0.0, max_value=49.9),
    current=st.floats(min_value=0.0, max_value=4.9),
    dist=st.floats(min_value=0.5, max_value=19.9),
)
@settings(max_examples=30)
def test_property_always_two_ships(wind, current, dist):
    """Property: 恶劣天气场景始终生成2艘船"""
    gen = ScenarioGenerator()
    p = RoughWeatherParams(
        wind_speed_ms=wind, current_speed_ms=current, distance_nm=dist)
    sc = gen.generate_rough_weather_scenario(p)
    assert len(sc.ships) == 2


@given(
    weather=st.sampled_from(['calm', 'moderate', 'rough']),
    vis=st.floats(min_value=0.1, max_value=20.0),
)
@settings(max_examples=30)
def test_property_visibility_enum_mapping(weather, vis):
    """Property: 能见度数值正确映射到枚举"""
    gen = ScenarioGenerator()
    p = RoughWeatherParams(weather_condition=weather, visibility_nm=vis)
    sc = gen.generate_rough_weather_scenario(p)
    if vis >= 5.0:
        assert sc.environment.visibility == Visibility.GOOD
    elif vis >= 2.0:
        assert sc.environment.visibility == Visibility.MODERATE
    else:
        assert sc.environment.visibility == Visibility.POOR


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
