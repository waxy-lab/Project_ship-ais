"""
多船包围场景生成器单元测试
Requirements: 2.2
"""
import math
import pytest
from hypothesis import given, settings, strategies as st

from scenario_generator.generator import ScenarioGenerator, SurroundingParams
from scenario_generator.models import ScenarioType


@pytest.fixture
def gen():
    return ScenarioGenerator()


class TestSurroundingParams:
    def test_default_valid(self):
        p = SurroundingParams()
        assert p.num_surrounding == 4
        assert p.surrounding_distance == 2.0

    def test_too_few_ships(self):
        with pytest.raises(ValueError):
            SurroundingParams(num_surrounding=1)

    def test_too_many_ships(self):
        with pytest.raises(ValueError):
            SurroundingParams(num_surrounding=13)

    def test_invalid_distance_zero(self):
        with pytest.raises(ValueError):
            SurroundingParams(surrounding_distance=0.0)

    def test_invalid_distance_too_large(self):
        with pytest.raises(ValueError):
            SurroundingParams(surrounding_distance=25.0)

    def test_invalid_own_speed(self):
        with pytest.raises(ValueError):
            SurroundingParams(own_speed=0.0)

    def test_min_speed_greater_than_max(self):
        with pytest.raises(ValueError):
            SurroundingParams(min_speed=20.0, max_speed=10.0)


class TestSurroundingScenario:
    def test_generates_scenario(self, gen):
        p = SurroundingParams(num_surrounding=4, seed=42)
        sc = gen.generate_surrounding_scenario(p)
        assert sc is not None

    def test_scenario_type_multi_ship(self, gen):
        p = SurroundingParams(num_surrounding=4, seed=42)
        sc = gen.generate_surrounding_scenario(p)
        assert sc.scenario_type == ScenarioType.MULTI_SHIP

    def test_ship_count_correct(self, gen):
        for n in [2, 3, 4, 6, 8]:
            p = SurroundingParams(num_surrounding=n, seed=0)
            sc = gen.generate_surrounding_scenario(p)
            # 本船 + n 艘包围船
            assert len(sc.ships) == n + 1

    def test_own_ship_at_center(self, gen):
        p = SurroundingParams(
            num_surrounding=4, seed=42,
            base_latitude=30.0, base_longitude=120.0
        )
        sc = gen.generate_surrounding_scenario(p)
        own = sc.ships[0]
        assert abs(own.latitude - 30.0) < 1e-9
        assert abs(own.longitude - 120.0) < 1e-9

    def test_own_ship_mmsi(self, gen):
        p = SurroundingParams(own_mmsi=999999999, seed=0)
        sc = gen.generate_surrounding_scenario(p)
        assert sc.ships[0].mmsi == 999999999

    def test_surrounding_ships_face_toward_own(self, gen):
        """包围船航向应大致朝向中心（本船）"""
        p = SurroundingParams(
            num_surrounding=4, seed=42,
            random_angles=False, random_offset=False,
            surrounding_distance=3.0
        )
        sc = gen.generate_surrounding_scenario(p)
        own = sc.ships[0]
        for ship in sc.ships[1:]:
            # 计算从包围船到本船的方位角
            dlat = own.latitude - ship.latitude
            dlon = own.longitude - ship.longitude
            bearing_to_own = math.degrees(math.atan2(dlon, dlat)) % 360
            hdiff = abs(((ship.heading - bearing_to_own + 180) % 360) - 180)
            assert hdiff < 5.0, f"包围船航向偏差过大: {hdiff:.1f}°"

    def test_surrounding_distance_approximately_correct(self, gen):
        """包围船到本船的距离应接近 surrounding_distance"""
        dist = 3.0
        p = SurroundingParams(
            num_surrounding=4, seed=0,
            surrounding_distance=dist,
            random_angles=False, random_offset=False
        )
        sc = gen.generate_surrounding_scenario(p)
        own = sc.ships[0]
        NM = 1 / 60.0
        for ship in sc.ships[1:]:
            dlat = (ship.latitude - own.latitude) / NM
            dlon = (ship.longitude - own.longitude) / NM * math.cos(
                math.radians(own.latitude))
            d = math.sqrt(dlat**2 + dlon**2)
            assert abs(d - dist) < 0.5, f"距离偏差过大: {d:.3f} vs {dist}"

    def test_uniform_angle_distribution(self, gen):
        """均匀分布时相邻方位角差应接近 360/n"""
        n = 4
        p = SurroundingParams(
            num_surrounding=n, seed=0,
            random_angles=False, random_offset=False,
            surrounding_distance=2.0
        )
        sc = gen.generate_surrounding_scenario(p)
        own = sc.ships[0]
        bearings = []
        for ship in sc.ships[1:]:
            dlat = ship.latitude - own.latitude
            dlon = ship.longitude - own.longitude
            b = math.degrees(math.atan2(dlon, dlat)) % 360
            bearings.append(b)
        bearings.sort()
        diffs = [(bearings[(i+1) % n] - bearings[i]) % 360
                 for i in range(n)]
        for d in diffs:
            assert abs(d - 360/n) < 5.0

    def test_random_seed_reproducible(self, gen):
        """相同种子生成相同场景"""
        p1 = SurroundingParams(num_surrounding=4, seed=123, random_angles=True)
        p2 = SurroundingParams(num_surrounding=4, seed=123, random_angles=True)
        sc1 = gen.generate_surrounding_scenario(p1)
        sc2 = gen.generate_surrounding_scenario(p2)
        for s1, s2 in zip(sc1.ships, sc2.ships):
            assert abs(s1.latitude - s2.latitude) < 1e-9
            assert abs(s1.longitude - s2.longitude) < 1e-9

    def test_different_seeds_different_scenarios(self, gen):
        """不同种子生成不同场景"""
        p1 = SurroundingParams(num_surrounding=4, seed=1, random_angles=True)
        p2 = SurroundingParams(num_surrounding=4, seed=2, random_angles=True)
        sc1 = gen.generate_surrounding_scenario(p1)
        sc2 = gen.generate_surrounding_scenario(p2)
        lats1 = [s.latitude for s in sc1.ships[1:]]
        lats2 = [s.latitude for s in sc2.ships[1:]]
        assert lats1 != lats2

    def test_custom_scenario_id(self, gen):
        p = SurroundingParams(scenario_id="surround_test_001")
        sc = gen.generate_surrounding_scenario(p)
        assert sc.scenario_id == "surround_test_001"

    def test_description_contains_count(self, gen):
        p = SurroundingParams(num_surrounding=6, seed=0)
        sc = gen.generate_surrounding_scenario(p)
        assert "6" in sc.description

    def test_all_ships_valid_heading(self, gen):
        p = SurroundingParams(num_surrounding=8, seed=42)
        sc = gen.generate_surrounding_scenario(p)
        for ship in sc.ships:
            assert 0.0 <= ship.heading < 360.0

    def test_all_ships_positive_speed(self, gen):
        p = SurroundingParams(num_surrounding=6, seed=42)
        sc = gen.generate_surrounding_scenario(p)
        for ship in sc.ships:
            assert ship.sog > 0.0

    def test_duration_set_correctly(self, gen):
        p = SurroundingParams(duration=900.0)
        sc = gen.generate_surrounding_scenario(p)
        assert abs(sc.duration - 900.0) < 1e-9


@given(
    n=st.integers(min_value=2, max_value=8),
    dist=st.floats(min_value=0.5, max_value=10.0),
    seed=st.integers(min_value=0, max_value=9999)
)
@settings(max_examples=30)
def test_property_surrounding_ship_count(n, dist, seed):
    """Property: 生成的船舶数量始终等于 num_surrounding + 1"""
    gen = ScenarioGenerator()
    p = SurroundingParams(num_surrounding=n, surrounding_distance=dist, seed=seed)
    sc = gen.generate_surrounding_scenario(p)
    assert len(sc.ships) == n + 1


@given(
    n=st.integers(min_value=2, max_value=6),
    seed=st.integers(min_value=0, max_value=9999)
)
@settings(max_examples=30)
def test_property_mmsi_unique(n, seed):
    """Property: 所有船舶的 MMSI 互不相同"""
    gen = ScenarioGenerator()
    p = SurroundingParams(num_surrounding=n, seed=seed)
    sc = gen.generate_surrounding_scenario(p)
    mmsis = [s.mmsi for s in sc.ships]
    assert len(mmsis) == len(set(mmsis))


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
