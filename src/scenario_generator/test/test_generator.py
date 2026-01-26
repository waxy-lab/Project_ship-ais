"""
场景生成器的单元测试

测试 ScenarioGenerator 的各种场景生成方法
"""

import pytest
import math
from hypothesis import given, settings
from hypothesis import strategies as st
from scenario_generator import (
    ScenarioGenerator,
    HeadOnParams,
    CrossingParams,
    ScenarioType,
    EnvironmentConfig,
)
from test_framework.strategies import head_on_params_strategy


class TestHeadOnScenarioGenerator:
    """测试对遇场景生成器"""
    
    def test_basic_head_on_generation(self):
        """测试基本对遇场景生成"""
        generator = ScenarioGenerator()
        params = HeadOnParams(
            distance=2.0,
            speed1=10.0,
            speed2=12.0,
            base_latitude=30.0,
            base_longitude=120.0
        )
        
        scenario = generator.generate_head_on_scenario(params)
        
        # 验证场景类型
        assert scenario.scenario_type == ScenarioType.HEAD_ON
        
        # 验证船舶数量
        assert len(scenario.ships) == 2
        
        ship1, ship2 = scenario.ships[0], scenario.ships[1]
        
        # 验证航向差（应该接近180度）
        heading_diff = abs(ship1.heading - ship2.heading)
        if heading_diff > 180:
            heading_diff = 360 - heading_diff
        
        assert 170 <= heading_diff <= 190, f"航向差应在170-190度之间，实际为{heading_diff}度"
        
        # 验证船舶在同一纬度（对遇场景）
        lat_diff = abs(ship1.latitude - ship2.latitude)
        assert lat_diff < 0.001, f"两船应在同一纬度，纬度差为{lat_diff}"
        
        # 验证船舶距离
        lon_diff = abs(ship1.longitude - ship2.longitude)
        distance_in_degrees = params.distance * (1.0 / 60.0)
        assert abs(lon_diff - distance_in_degrees) < 0.001, "船舶距离不正确"
        
        # 验证速度
        assert ship1.sog == params.speed1
        assert ship2.sog == params.speed2
    
    def test_head_on_geometry(self):
        """测试对遇场景的几何正确性"""
        generator = ScenarioGenerator()
        params = HeadOnParams(
            distance=5.0,
            speed1=15.0,
            speed2=15.0
        )
        
        scenario = generator.generate_head_on_scenario(params)
        ship1, ship2 = scenario.ships[0], scenario.ships[1]
        
        # 计算相对方位
        # 从船1看船2的方位
        lon_diff = ship2.longitude - ship1.longitude
        lat_diff = ship2.latitude - ship1.latitude
        
        # 计算方位角（从北顺时针）
        bearing = math.degrees(math.atan2(lon_diff, lat_diff))
        if bearing < 0:
            bearing += 360
        
        # 计算相对方位（方位角 - 航向）
        relative_bearing = bearing - ship1.heading
        if relative_bearing > 180:
            relative_bearing -= 360
        elif relative_bearing < -180:
            relative_bearing += 360
        
        # 对于对遇场景，相对方位应该接近0度（正前方）
        assert -10 <= relative_bearing <= 10, \
            f"相对方位应在-10到10度之间，实际为{relative_bearing}度"
    
    def test_head_on_with_different_speeds(self):
        """测试不同速度的对遇场景"""
        generator = ScenarioGenerator()
        params = HeadOnParams(
            distance=3.0,
            speed1=8.0,
            speed2=20.0
        )
        
        scenario = generator.generate_head_on_scenario(params)
        ship1, ship2 = scenario.ships[0], scenario.ships[1]
        
        assert ship1.sog == 8.0
        assert ship2.sog == 20.0
        
        # 验证仍然满足对遇几何条件
        heading_diff = abs(ship1.heading - ship2.heading)
        if heading_diff > 180:
            heading_diff = 360 - heading_diff
        
        assert 170 <= heading_diff <= 190
    
    def test_head_on_with_custom_location(self):
        """测试自定义位置的对遇场景"""
        generator = ScenarioGenerator()
        params = HeadOnParams(
            distance=2.0,
            speed1=10.0,
            speed2=10.0,
            base_latitude=45.0,
            base_longitude=130.0
        )
        
        scenario = generator.generate_head_on_scenario(params)
        ship1, ship2 = scenario.ships[0], scenario.ships[1]
        
        # 验证船舶位置在基准位置附近
        avg_lat = (ship1.latitude + ship2.latitude) / 2
        avg_lon = (ship1.longitude + ship2.longitude) / 2
        
        assert abs(avg_lat - params.base_latitude) < 0.001
        assert abs(avg_lon - params.base_longitude) < 0.001
    
    def test_head_on_with_custom_mmsi(self):
        """测试自定义MMSI的对遇场景"""
        generator = ScenarioGenerator()
        params = HeadOnParams(
            distance=2.0,
            speed1=10.0,
            speed2=10.0,
            mmsi1=111111111,
            mmsi2=222222222
        )
        
        scenario = generator.generate_head_on_scenario(params)
        ship1, ship2 = scenario.ships[0], scenario.ships[1]
        
        assert ship1.mmsi == 111111111
        assert ship2.mmsi == 222222222
    
    def test_head_on_scenario_metadata(self):
        """测试对遇场景的元数据"""
        generator = ScenarioGenerator()
        params = HeadOnParams(
            distance=2.0,
            speed1=10.0,
            speed2=12.0
        )
        
        scenario = generator.generate_head_on_scenario(params)
        
        # 验证场景ID存在且非空
        assert scenario.scenario_id
        assert scenario.scenario_id.startswith("head_on_")
        
        # 验证场景描述
        assert scenario.description
        assert "对遇场景" in scenario.description
        
        # 验证成功标准
        assert 'min_distance' in scenario.success_criteria
        assert 'collision_avoided' in scenario.success_criteria
        
        # 验证持续时间
        assert scenario.duration == 600.0
    
    def test_head_on_with_custom_environment(self):
        """测试带自定义环境的对遇场景"""
        from scenario_generator import WeatherCondition, Visibility
        
        custom_env = EnvironmentConfig(
            weather_condition=WeatherCondition.ROUGH,
            visibility=Visibility.POOR,
            wind_speed=15.0,
            wind_direction=90.0
        )
        
        generator = ScenarioGenerator(environment=custom_env)
        params = HeadOnParams(
            distance=2.0,
            speed1=10.0,
            speed2=10.0
        )
        
        scenario = generator.generate_head_on_scenario(params)
        
        assert scenario.environment.weather_condition == WeatherCondition.ROUGH
        assert scenario.environment.visibility == Visibility.POOR
        assert scenario.environment.wind_speed == 15.0


class TestHeadOnParamsValidation:
    """测试对遇场景参数验证"""
    
    def test_invalid_distance_negative(self):
        """测试负距离"""
        with pytest.raises(ValueError, match="距离必须大于0"):
            HeadOnParams(distance=-1.0, speed1=10.0, speed2=10.0)
    
    def test_invalid_distance_zero(self):
        """测试距离为0"""
        with pytest.raises(ValueError, match="距离必须大于0"):
            HeadOnParams(distance=0.0, speed1=10.0, speed2=10.0)
    
    def test_invalid_distance_too_large(self):
        """测试距离过大"""
        with pytest.raises(ValueError, match="距离不应超过20海里"):
            HeadOnParams(distance=21.0, speed1=10.0, speed2=10.0)
    
    def test_invalid_speed1_negative(self):
        """测试船1速度为负"""
        with pytest.raises(ValueError, match="船1速度必须大于0"):
            HeadOnParams(distance=2.0, speed1=-5.0, speed2=10.0)
    
    def test_invalid_speed1_zero(self):
        """测试船1速度为0"""
        with pytest.raises(ValueError, match="船1速度必须大于0"):
            HeadOnParams(distance=2.0, speed1=0.0, speed2=10.0)
    
    def test_invalid_speed2_negative(self):
        """测试船2速度为负"""
        with pytest.raises(ValueError, match="船2速度必须大于0"):
            HeadOnParams(distance=2.0, speed1=10.0, speed2=-5.0)
    
    def test_invalid_speed_too_high(self):
        """测试速度过大"""
        with pytest.raises(ValueError, match="速度不应超过50节"):
            HeadOnParams(distance=2.0, speed1=51.0, speed2=10.0)
    
    def test_invalid_latitude_too_low(self):
        """测试纬度过低"""
        with pytest.raises(ValueError, match="纬度必须在-90到90之间"):
            HeadOnParams(
                distance=2.0,
                speed1=10.0,
                speed2=10.0,
                base_latitude=-91.0
            )
    
    def test_invalid_latitude_too_high(self):
        """测试纬度过高"""
        with pytest.raises(ValueError, match="纬度必须在-90到90之间"):
            HeadOnParams(
                distance=2.0,
                speed1=10.0,
                speed2=10.0,
                base_latitude=91.0
            )
    
    def test_invalid_longitude_too_low(self):
        """测试经度过低"""
        with pytest.raises(ValueError, match="经度必须在-180到180之间"):
            HeadOnParams(
                distance=2.0,
                speed1=10.0,
                speed2=10.0,
                base_longitude=-181.0
            )
    
    def test_invalid_longitude_too_high(self):
        """测试经度过高"""
        with pytest.raises(ValueError, match="经度必须在-180到180之间"):
            HeadOnParams(
                distance=2.0,
                speed1=10.0,
                speed2=10.0,
                base_longitude=181.0
            )
    
    def test_valid_boundary_values(self):
        """测试边界值"""
        # 最小有效距离
        params = HeadOnParams(distance=0.1, speed1=5.0, speed2=5.0)
        assert params.distance == 0.1
        
        # 最大有效距离
        params = HeadOnParams(distance=20.0, speed1=5.0, speed2=5.0)
        assert params.distance == 20.0
        
        # 最大有效速度
        params = HeadOnParams(distance=2.0, speed1=50.0, speed2=50.0)
        assert params.speed1 == 50.0
        
        # 边界纬度
        params = HeadOnParams(
            distance=2.0,
            speed1=10.0,
            speed2=10.0,
            base_latitude=-90.0
        )
        assert params.base_latitude == -90.0
        
        params = HeadOnParams(
            distance=2.0,
            speed1=10.0,
            speed2=10.0,
            base_latitude=90.0
        )
        assert params.base_latitude == 90.0
        
        # 边界经度
        params = HeadOnParams(
            distance=2.0,
            speed1=10.0,
            speed2=10.0,
            base_longitude=-180.0
        )
        assert params.base_longitude == -180.0
        
        params = HeadOnParams(
            distance=2.0,
            speed1=10.0,
            speed2=10.0,
            base_longitude=180.0
        )
        assert params.base_longitude == 180.0


class TestScenarioGeneratorEdgeCases:
    """测试场景生成器的边界情况"""
    
    def test_very_small_distance(self):
        """测试非常小的距离"""
        generator = ScenarioGenerator()
        params = HeadOnParams(
            distance=0.1,  # 0.1海里
            speed1=10.0,
            speed2=10.0
        )
        
        scenario = generator.generate_head_on_scenario(params)
        ship1, ship2 = scenario.ships[0], scenario.ships[1]
        
        # 验证距离计算正确
        lon_diff = abs(ship1.longitude - ship2.longitude)
        expected_diff = 0.1 * (1.0 / 60.0)
        assert abs(lon_diff - expected_diff) < 0.0001
    
    def test_very_large_distance(self):
        """测试较大的距离"""
        generator = ScenarioGenerator()
        params = HeadOnParams(
            distance=20.0,  # 20海里（最大值）
            speed1=10.0,
            speed2=10.0
        )
        
        scenario = generator.generate_head_on_scenario(params)
        ship1, ship2 = scenario.ships[0], scenario.ships[1]
        
        # 验证距离计算正确
        lon_diff = abs(ship1.longitude - ship2.longitude)
        expected_diff = 20.0 * (1.0 / 60.0)
        assert abs(lon_diff - expected_diff) < 0.0001
    
    def test_very_slow_speed(self):
        """测试非常慢的速度"""
        generator = ScenarioGenerator()
        params = HeadOnParams(
            distance=2.0,
            speed1=0.1,  # 0.1节
            speed2=0.1
        )
        
        scenario = generator.generate_head_on_scenario(params)
        ship1, ship2 = scenario.ships[0], scenario.ships[1]
        
        assert ship1.sog == 0.1
        assert ship2.sog == 0.1
    
    def test_very_fast_speed(self):
        """测试非常快的速度"""
        generator = ScenarioGenerator()
        params = HeadOnParams(
            distance=2.0,
            speed1=50.0,  # 50节（最大值）
            speed2=50.0
        )
        
        scenario = generator.generate_head_on_scenario(params)
        ship1, ship2 = scenario.ships[0], scenario.ships[1]
        
        assert ship1.sog == 50.0
        assert ship2.sog == 50.0
    
    def test_extreme_latitude_north(self):
        """测试极北纬度"""
        generator = ScenarioGenerator()
        params = HeadOnParams(
            distance=2.0,
            speed1=10.0,
            speed2=10.0,
            base_latitude=89.0  # 接近北极
        )
        
        scenario = generator.generate_head_on_scenario(params)
        ship1, ship2 = scenario.ships[0], scenario.ships[1]
        
        # 验证船舶位置有效
        assert -90 <= ship1.latitude <= 90
        assert -90 <= ship2.latitude <= 90
    
    def test_extreme_latitude_south(self):
        """测试极南纬度"""
        generator = ScenarioGenerator()
        params = HeadOnParams(
            distance=2.0,
            speed1=10.0,
            speed2=10.0,
            base_latitude=-89.0  # 接近南极
        )
        
        scenario = generator.generate_head_on_scenario(params)
        ship1, ship2 = scenario.ships[0], scenario.ships[1]
        
        # 验证船舶位置有效
        assert -90 <= ship1.latitude <= 90
        assert -90 <= ship2.latitude <= 90
    
    def test_date_line_crossing(self):
        """测试跨越日期变更线"""
        generator = ScenarioGenerator()
        params = HeadOnParams(
            distance=2.0,
            speed1=10.0,
            speed2=10.0,
            base_longitude=179.0  # 接近日期变更线
        )
        
        scenario = generator.generate_head_on_scenario(params)
        ship1, ship2 = scenario.ships[0], scenario.ships[1]
        
        # 验证船舶位置有效（可能跨越180度）
        assert -180 <= ship1.longitude <= 180
        assert -180 <= ship2.longitude <= 180



# ============================================================================
# 基于属性的测试 (Property-Based Tests)
# ============================================================================

class TestHeadOnScenarioProperties:
    """
    对遇场景生成的属性测试
    
    使用 Hypothesis 进行基于属性的测试，验证场景生成的通用正确性
    """
    
    @given(
        distance=st.floats(min_value=0.5, max_value=10.0, allow_nan=False, allow_infinity=False),
        speed1=st.floats(min_value=5.0, max_value=20.0, allow_nan=False, allow_infinity=False),
        speed2=st.floats(min_value=5.0, max_value=20.0, allow_nan=False, allow_infinity=False),
        base_latitude=st.floats(min_value=20.0, max_value=50.0, allow_nan=False, allow_infinity=False),
        base_longitude=st.floats(min_value=100.0, max_value=140.0, allow_nan=False, allow_infinity=False)
    )
    @settings(max_examples=100, deadline=None)
    def test_property_head_on_geometry(self, distance, speed1, speed2, base_latitude, base_longitude):
        """
        **Property 1: 场景生成的几何正确性**
        **Validates: Requirements 1.1**
        
        Feature: maritime-collision-avoidance, Property 1: 场景生成的几何正确性
        
        对于任何对遇场景参数，生成的两艘船舶应满足对遇的几何条件：
        1. 航向差在170-190度之间
        2. 相对方位在-10到10度之间
        
        这个属性测试使用 Hypothesis 生成随机参数，验证所有情况下几何条件都成立。
        """
        # 创建场景生成器
        generator = ScenarioGenerator()
        
        # 创建参数
        params = HeadOnParams(
            distance=distance,
            speed1=speed1,
            speed2=speed2,
            base_latitude=base_latitude,
            base_longitude=base_longitude
        )
        
        # 生成场景
        scenario = generator.generate_head_on_scenario(params)
        
        # 验证场景类型
        assert scenario.scenario_type == ScenarioType.HEAD_ON, \
            "场景类型应为 HEAD_ON"
        
        # 验证船舶数量
        assert len(scenario.ships) == 2, \
            "对遇场景应包含2艘船舶"
        
        ship1, ship2 = scenario.ships[0], scenario.ships[1]
        
        # ========================================
        # 属性1：航向差在170-190度之间
        # ========================================
        heading_diff = abs(ship1.heading - ship2.heading)
        # 处理跨越0度的情况
        if heading_diff > 180:
            heading_diff = 360 - heading_diff
        
        assert 170 <= heading_diff <= 190, \
            f"航向差应在170-190度之间，实际为 {heading_diff:.2f} 度 " \
            f"(ship1: {ship1.heading:.2f}°, ship2: {ship2.heading:.2f}°)"
        
        # ========================================
        # 属性2：相对方位在-10到10度之间
        # ========================================
        # 计算从船1看船2的方位
        lon_diff = ship2.longitude - ship1.longitude
        lat_diff = ship2.latitude - ship1.latitude
        
        # 计算方位角（从北顺时针，使用 atan2）
        bearing = math.degrees(math.atan2(lon_diff, lat_diff))
        if bearing < 0:
            bearing += 360
        
        # 计算相对方位（方位角 - 航向）
        relative_bearing = bearing - ship1.heading
        
        # 归一化到 -180 到 180 度范围
        if relative_bearing > 180:
            relative_bearing -= 360
        elif relative_bearing < -180:
            relative_bearing += 360
        
        assert -10 <= relative_bearing <= 10, \
            f"相对方位应在-10到10度之间，实际为 {relative_bearing:.2f} 度 " \
            f"(bearing: {bearing:.2f}°, ship1 heading: {ship1.heading:.2f}°)"
        
        # ========================================
        # 额外验证：船舶速度正确
        # ========================================
        assert ship1.sog == pytest.approx(speed1, rel=1e-6), \
            f"船1速度应为 {speed1} 节"
        assert ship2.sog == pytest.approx(speed2, rel=1e-6), \
            f"船2速度应为 {speed2} 节"
        
        # ========================================
        # 额外验证：船舶距离正确
        # ========================================
        # 计算实际距离（简化为经度差，因为纬度相同）
        actual_lon_diff = abs(ship1.longitude - ship2.longitude)
        expected_lon_diff = distance * (1.0 / 60.0)  # 海里转度
        
        assert abs(actual_lon_diff - expected_lon_diff) < 0.001, \
            f"船舶距离应为 {distance} 海里，实际经度差为 {actual_lon_diff:.6f} 度"
    
    @given(head_on_params_strategy())
    @settings(max_examples=100, deadline=None)
    def test_property_head_on_geometry_with_strategy(self, params):
        """
        **Property 1: 场景生成的几何正确性（使用策略）**
        **Validates: Requirements 1.1**
        
        Feature: maritime-collision-avoidance, Property 1: 场景生成的几何正确性
        
        使用预定义的 Hypothesis 策略生成参数，验证几何正确性。
        这是对上面测试的补充，使用了更高级的策略组合。
        """
        generator = ScenarioGenerator()
        
        # 从策略字典创建参数对象
        head_on_params = HeadOnParams(
            distance=params['distance'],
            speed1=params['speed1'],
            speed2=params['speed2'],
            base_latitude=params['base_lat'],
            base_longitude=params['base_lon']
        )
        
        # 生成场景
        scenario = generator.generate_head_on_scenario(head_on_params)
        
        ship1, ship2 = scenario.ships[0], scenario.ships[1]
        
        # 验证航向差
        heading_diff = abs(ship1.heading - ship2.heading)
        if heading_diff > 180:
            heading_diff = 360 - heading_diff
        
        assert 170 <= heading_diff <= 190, \
            f"航向差应在170-190度之间，实际为 {heading_diff:.2f} 度"
        
        # 验证相对方位
        lon_diff = ship2.longitude - ship1.longitude
        lat_diff = ship2.latitude - ship1.latitude
        bearing = math.degrees(math.atan2(lon_diff, lat_diff))
        if bearing < 0:
            bearing += 360
        
        relative_bearing = bearing - ship1.heading
        if relative_bearing > 180:
            relative_bearing -= 360
        elif relative_bearing < -180:
            relative_bearing += 360
        
        assert -10 <= relative_bearing <= 10, \
            f"相对方位应在-10到10度之间，实际为 {relative_bearing:.2f} 度"
    
    @given(
        distance=st.floats(min_value=0.5, max_value=10.0, allow_nan=False, allow_infinity=False),
        speed1=st.floats(min_value=5.0, max_value=20.0, allow_nan=False, allow_infinity=False),
        speed2=st.floats(min_value=5.0, max_value=20.0, allow_nan=False, allow_infinity=False)
    )
    @settings(max_examples=100, deadline=None)
    def test_property_head_on_ships_on_collision_course(self, distance, speed1, speed2):
        """
        **Property 1 扩展：对遇场景的碰撞航线**
        **Validates: Requirements 1.1**
        
        Feature: maritime-collision-avoidance, Property 1: 场景生成的几何正确性
        
        验证生成的对遇场景中，两艘船舶确实在碰撞航线上（如果不改变航向）。
        这通过验证两船的航迹会在某个点相交来实现。
        """
        generator = ScenarioGenerator()
        
        params = HeadOnParams(
            distance=distance,
            speed1=speed1,
            speed2=speed2
        )
        
        scenario = generator.generate_head_on_scenario(params)
        ship1, ship2 = scenario.ships[0], scenario.ships[1]
        
        # 验证两船在同一纬度（对于东西向对遇）
        lat_diff = abs(ship1.latitude - ship2.latitude)
        assert lat_diff < 0.001, \
            f"对遇场景中两船应在同一纬度，纬度差为 {lat_diff:.6f}"
        
        # 验证两船相向航行（一个向东，一个向西）
        # 船1航向应为90度（向东），船2航向应为270度（向西）
        assert ship1.heading == pytest.approx(90.0, abs=0.1), \
            f"船1应向东航行（90度），实际航向 {ship1.heading:.2f} 度"
        assert ship2.heading == pytest.approx(270.0, abs=0.1), \
            f"船2应向西航行（270度），实际航向 {ship2.heading:.2f} 度"
        
        # 验证船1在船2的西侧
        assert ship1.longitude < ship2.longitude, \
            "船1应在船2的西侧"
    
    @given(
        distance=st.floats(min_value=0.5, max_value=10.0, allow_nan=False, allow_infinity=False),
        speed1=st.floats(min_value=5.0, max_value=20.0, allow_nan=False, allow_infinity=False),
        speed2=st.floats(min_value=5.0, max_value=20.0, allow_nan=False, allow_infinity=False)
    )
    @settings(max_examples=100, deadline=None)
    def test_property_head_on_scenario_consistency(self, distance, speed1, speed2):
        """
        **Property 1 扩展：场景生成的一致性**
        **Validates: Requirements 1.1**
        
        Feature: maritime-collision-avoidance, Property 1: 场景生成的几何正确性
        
        验证使用相同参数多次生成场景时，结果应该一致（除了随机的场景ID）。
        """
        generator = ScenarioGenerator()
        
        params = HeadOnParams(
            distance=distance,
            speed1=speed1,
            speed2=speed2,
            base_latitude=30.0,
            base_longitude=120.0,
            mmsi1=123456789,
            mmsi2=987654321
        )
        
        # 生成两次场景
        scenario1 = generator.generate_head_on_scenario(params)
        scenario2 = generator.generate_head_on_scenario(params)
        
        # 验证船舶状态一致（除了场景ID）
        ship1_a, ship2_a = scenario1.ships[0], scenario1.ships[1]
        ship1_b, ship2_b = scenario2.ships[0], scenario2.ships[1]
        
        # 船1状态应该一致
        assert ship1_a.mmsi == ship1_b.mmsi
        assert ship1_a.latitude == pytest.approx(ship1_b.latitude, abs=1e-9)
        assert ship1_a.longitude == pytest.approx(ship1_b.longitude, abs=1e-9)
        assert ship1_a.heading == pytest.approx(ship1_b.heading, abs=1e-9)
        assert ship1_a.sog == pytest.approx(ship1_b.sog, abs=1e-9)
        
        # 船2状态应该一致
        assert ship2_a.mmsi == ship2_b.mmsi
        assert ship2_a.latitude == pytest.approx(ship2_b.latitude, abs=1e-9)
        assert ship2_a.longitude == pytest.approx(ship2_b.longitude, abs=1e-9)
        assert ship2_a.heading == pytest.approx(ship2_b.heading, abs=1e-9)
        assert ship2_a.sog == pytest.approx(ship2_b.sog, abs=1e-9)


# ============================================================================
# 交叉场景测试
# ============================================================================

class TestCrossingScenarioGenerator:
    """测试交叉相遇场景生成器"""
    
    def test_basic_crossing_generation(self):
        """测试基本交叉场景生成"""
        generator = ScenarioGenerator()
        params = CrossingParams(
            distance=2.0,
            speed1=10.0,
            speed2=12.0,
            crossing_angle=45.0,
            base_latitude=30.0,
            base_longitude=120.0
        )
        
        scenario = generator.generate_crossing_scenario(params)
        
        # 验证场景类型
        assert scenario.scenario_type == ScenarioType.CROSSING
        
        # 验证船舶数量
        assert len(scenario.ships) == 2
        
        ship1, ship2 = scenario.ships[0], scenario.ships[1]
        
        # 验证速度
        assert ship1.sog == params.speed1
        assert ship2.sog == params.speed2
    
    def test_crossing_with_positive_angle(self):
        """测试正角度的交叉场景"""
        generator = ScenarioGenerator()
        params = CrossingParams(
            distance=3.0,
            speed1=10.0,
            speed2=10.0,
            crossing_angle=60.0
        )
        
        scenario = generator.generate_crossing_scenario(params)
        ship1, ship2 = scenario.ships[0], scenario.ships[1]
        
        # 计算从船1看船2的相对方位
        lon_diff = ship2.longitude - ship1.longitude
        lat_diff = ship2.latitude - ship1.latitude
        
        bearing = math.degrees(math.atan2(lon_diff, lat_diff))
        if bearing < 0:
            bearing += 360
        
        relative_bearing = bearing - ship1.heading
        if relative_bearing > 180:
            relative_bearing -= 360
        elif relative_bearing < -180:
            relative_bearing += 360
        
        # 验证相对方位在有效范围内
        assert 5 <= abs(relative_bearing) <= 112.5, \
            f"相对方位应在5-112.5度范围内，实际为{relative_bearing:.2f}度"
    
    def test_crossing_with_negative_angle(self):
        """测试负角度的交叉场景"""
        generator = ScenarioGenerator()
        params = CrossingParams(
            distance=2.0,
            speed1=10.0,
            speed2=10.0,
            crossing_angle=-45.0
        )
        
        scenario = generator.generate_crossing_scenario(params)
        ship1, ship2 = scenario.ships[0], scenario.ships[1]
        
        # 计算从船1看船2的相对方位
        lon_diff = ship2.longitude - ship1.longitude
        lat_diff = ship2.latitude - ship1.latitude
        
        bearing = math.degrees(math.atan2(lon_diff, lat_diff))
        if bearing < 0:
            bearing += 360
        
        relative_bearing = bearing - ship1.heading
        if relative_bearing > 180:
            relative_bearing -= 360
        elif relative_bearing < -180:
            relative_bearing += 360
        
        # 验证相对方位在有效范围内
        assert 5 <= abs(relative_bearing) <= 112.5, \
            f"相对方位应在5-112.5度范围内，实际为{relative_bearing:.2f}度"


class TestCrossingParamsValidation:
    """测试交叉场景参数验证"""
    
    def test_invalid_crossing_angle_too_small(self):
        """测试交叉角度过小"""
        with pytest.raises(ValueError, match="交叉角度必须在5-112.5度或-112.5到-5度之间"):
            CrossingParams(
                distance=2.0,
                speed1=10.0,
                speed2=10.0,
                crossing_angle=3.0
            )
    
    def test_invalid_crossing_angle_in_forbidden_range(self):
        """测试交叉角度在禁止范围内（-5到5度）"""
        with pytest.raises(ValueError, match="交叉角度必须在5-112.5度或-112.5到-5度之间"):
            CrossingParams(
                distance=2.0,
                speed1=10.0,
                speed2=10.0,
                crossing_angle=0.0
            )
    
    def test_invalid_crossing_angle_too_large(self):
        """测试交叉角度过大"""
        with pytest.raises(ValueError, match="交叉角度必须在5-112.5度或-112.5到-5度之间"):
            CrossingParams(
                distance=2.0,
                speed1=10.0,
                speed2=10.0,
                crossing_angle=120.0
            )
    
    def test_valid_crossing_angle_boundaries(self):
        """测试交叉角度边界值"""
        # 正向最小值
        params = CrossingParams(
            distance=2.0,
            speed1=10.0,
            speed2=10.0,
            crossing_angle=5.0
        )
        assert params.crossing_angle == 5.0
        
        # 正向最大值
        params = CrossingParams(
            distance=2.0,
            speed1=10.0,
            speed2=10.0,
            crossing_angle=112.5
        )
        assert params.crossing_angle == 112.5
        
        # 负向最小值
        params = CrossingParams(
            distance=2.0,
            speed1=10.0,
            speed2=10.0,
            crossing_angle=-5.0
        )
        assert params.crossing_angle == -5.0
        
        # 负向最大值
        params = CrossingParams(
            distance=2.0,
            speed1=10.0,
            speed2=10.0,
            crossing_angle=-112.5
        )
        assert params.crossing_angle == -112.5


# ============================================================================
# 交叉场景属性测试
# ============================================================================

class TestCrossingScenarioProperties:
    """
    交叉相遇场景生成的属性测试
    
    使用 Hypothesis 进行基于属性的测试，验证场景生成的通用正确性
    """
    
    @given(
        distance=st.floats(min_value=0.5, max_value=10.0, allow_nan=False, allow_infinity=False),
        speed1=st.floats(min_value=5.0, max_value=20.0, allow_nan=False, allow_infinity=False),
        speed2=st.floats(min_value=5.0, max_value=20.0, allow_nan=False, allow_infinity=False),
        crossing_angle=st.one_of(
            st.floats(min_value=5.0, max_value=112.5, allow_nan=False, allow_infinity=False),
            st.floats(min_value=-112.5, max_value=-5.0, allow_nan=False, allow_infinity=False)
        ),
        base_latitude=st.floats(min_value=20.0, max_value=50.0, allow_nan=False, allow_infinity=False),
        base_longitude=st.floats(min_value=100.0, max_value=140.0, allow_nan=False, allow_infinity=False)
    )
    @settings(max_examples=100, deadline=None)
    def test_property_crossing_angle_constraint(
        self, distance, speed1, speed2, crossing_angle, base_latitude, base_longitude
    ):
        """
        **Property 2: 交叉场景的角度约束**
        **Validates: Requirements 1.2**
        
        Feature: maritime-collision-avoidance, Property 2: 交叉场景的角度约束
        
        对于任何交叉相遇场景参数，生成的两艘船舶的相对方位应在
        5-112.5度或-112.5到-5度之间。
        
        这个属性测试使用 Hypothesis 生成随机参数，验证所有情况下角度约束都成立。
        """
        # 创建场景生成器
        generator = ScenarioGenerator()
        
        # 创建参数
        params = CrossingParams(
            distance=distance,
            speed1=speed1,
            speed2=speed2,
            crossing_angle=crossing_angle,
            base_latitude=base_latitude,
            base_longitude=base_longitude
        )
        
        # 生成场景
        scenario = generator.generate_crossing_scenario(params)
        
        # 验证场景类型
        assert scenario.scenario_type == ScenarioType.CROSSING, \
            "场景类型应为 CROSSING"
        
        # 验证船舶数量
        assert len(scenario.ships) == 2, \
            "交叉场景应包含2艘船舶"
        
        ship1, ship2 = scenario.ships[0], scenario.ships[1]
        
        # ========================================
        # 属性2：相对方位在5-112.5度或-112.5到-5度之间
        # ========================================
        # 计算从船1看船2的方位
        lon_diff = ship2.longitude - ship1.longitude
        lat_diff = ship2.latitude - ship1.latitude
        
        # 计算方位角（从北顺时针，使用 atan2）
        bearing = math.degrees(math.atan2(lon_diff, lat_diff))
        if bearing < 0:
            bearing += 360
        
        # 计算相对方位（方位角 - 航向）
        relative_bearing = bearing - ship1.heading
        
        # 归一化到 -180 到 180 度范围
        if relative_bearing > 180:
            relative_bearing -= 360
        elif relative_bearing < -180:
            relative_bearing += 360
        
        # 验证相对方位在有效范围内
        # 允许小的浮点数误差（0.1度）
        tolerance = 0.1
        is_valid_positive = (5 - tolerance) <= relative_bearing <= (112.5 + tolerance)
        is_valid_negative = (-112.5 - tolerance) <= relative_bearing <= (-5 + tolerance)
        
        assert is_valid_positive or is_valid_negative, \
            f"相对方位应在5-112.5度或-112.5到-5度之间，实际为 {relative_bearing:.2f} 度 " \
            f"(bearing: {bearing:.2f}°, ship1 heading: {ship1.heading:.2f}°, " \
            f"crossing_angle: {crossing_angle:.2f}°)"
        
        # ========================================
        # 额外验证：船舶速度正确
        # ========================================
        assert ship1.sog == pytest.approx(speed1, rel=1e-6), \
            f"船1速度应为 {speed1} 节"
        assert ship2.sog == pytest.approx(speed2, rel=1e-6), \
            f"船2速度应为 {speed2} 节"
        
        # ========================================
        # 额外验证：船舶距离正确
        # ========================================
        # 计算实际距离（使用Haversine公式的简化版本）
        actual_distance_degrees = math.sqrt(
            (ship2.latitude - ship1.latitude)**2 + 
            (ship2.longitude - ship1.longitude)**2
        )
        expected_distance_degrees = distance * (1.0 / 60.0)  # 海里转度
        
        # 允许一定的误差（因为地球曲率等因素）
        assert abs(actual_distance_degrees - expected_distance_degrees) < 0.01, \
            f"船舶距离应约为 {distance} 海里，实际距离为 {actual_distance_degrees * 60:.2f} 海里"
    
    @given(
        distance=st.floats(min_value=0.5, max_value=10.0, allow_nan=False, allow_infinity=False),
        speed1=st.floats(min_value=5.0, max_value=20.0, allow_nan=False, allow_infinity=False),
        speed2=st.floats(min_value=5.0, max_value=20.0, allow_nan=False, allow_infinity=False),
        crossing_angle=st.one_of(
            st.floats(min_value=5.0, max_value=112.5, allow_nan=False, allow_infinity=False),
            st.floats(min_value=-112.5, max_value=-5.0, allow_nan=False, allow_infinity=False)
        )
    )
    @settings(max_examples=100, deadline=None)
    def test_property_crossing_ships_will_meet(
        self, distance, speed1, speed2, crossing_angle
    ):
        """
        **Property 2 扩展：交叉场景的航线相交**
        **Validates: Requirements 1.2**
        
        Feature: maritime-collision-avoidance, Property 2: 交叉场景的角度约束
        
        验证生成的交叉场景中，两艘船舶的航线会相交（如果不改变航向）。
        这通过验证两船的航迹会在某个点相遇来实现。
        """
        generator = ScenarioGenerator()
        
        params = CrossingParams(
            distance=distance,
            speed1=speed1,
            speed2=speed2,
            crossing_angle=crossing_angle
        )
        
        scenario = generator.generate_crossing_scenario(params)
        ship1, ship2 = scenario.ships[0], scenario.ships[1]
        
        # 验证两船不在同一位置
        position_diff = math.sqrt(
            (ship2.latitude - ship1.latitude)**2 + 
            (ship2.longitude - ship1.longitude)**2
        )
        assert position_diff > 0.001, \
            "两船初始位置应该不同"
        
        # 验证两船航向不同（不是平行航行）
        heading_diff = abs(ship1.heading - ship2.heading)
        if heading_diff > 180:
            heading_diff = 360 - heading_diff
        
        # 交叉场景中，航向差不应接近0度或180度
        assert not (heading_diff < 10 or heading_diff > 170), \
            f"交叉场景中航向差应在10-170度之间，实际为 {heading_diff:.2f} 度"
    
    @given(
        distance=st.floats(min_value=0.5, max_value=10.0, allow_nan=False, allow_infinity=False),
        speed1=st.floats(min_value=5.0, max_value=20.0, allow_nan=False, allow_infinity=False),
        speed2=st.floats(min_value=5.0, max_value=20.0, allow_nan=False, allow_infinity=False),
        crossing_angle=st.one_of(
            st.floats(min_value=5.0, max_value=112.5, allow_nan=False, allow_infinity=False),
            st.floats(min_value=-112.5, max_value=-5.0, allow_nan=False, allow_infinity=False)
        )
    )
    @settings(max_examples=100, deadline=None)
    def test_property_crossing_scenario_consistency(
        self, distance, speed1, speed2, crossing_angle
    ):
        """
        **Property 2 扩展：交叉场景生成的一致性**
        **Validates: Requirements 1.2**
        
        Feature: maritime-collision-avoidance, Property 2: 交叉场景的角度约束
        
        验证使用相同参数多次生成场景时，结果应该一致（除了随机的场景ID）。
        """
        generator = ScenarioGenerator()
        
        params = CrossingParams(
            distance=distance,
            speed1=speed1,
            speed2=speed2,
            crossing_angle=crossing_angle,
            base_latitude=30.0,
            base_longitude=120.0,
            mmsi1=123456789,
            mmsi2=987654321
        )
        
        # 生成两次场景
        scenario1 = generator.generate_crossing_scenario(params)
        scenario2 = generator.generate_crossing_scenario(params)
        
        # 验证船舶状态一致（除了场景ID）
        ship1_a, ship2_a = scenario1.ships[0], scenario1.ships[1]
        ship1_b, ship2_b = scenario2.ships[0], scenario2.ships[1]
        
        # 船1状态应该一致
        assert ship1_a.mmsi == ship1_b.mmsi
        assert ship1_a.latitude == pytest.approx(ship1_b.latitude, abs=1e-9)
        assert ship1_a.longitude == pytest.approx(ship1_b.longitude, abs=1e-9)
        assert ship1_a.heading == pytest.approx(ship1_b.heading, abs=1e-9)
        assert ship1_a.sog == pytest.approx(ship1_b.sog, abs=1e-9)
        
        # 船2状态应该一致
        assert ship2_a.mmsi == ship2_b.mmsi
        assert ship2_a.latitude == pytest.approx(ship2_b.latitude, abs=1e-9)
        assert ship2_a.longitude == pytest.approx(ship2_b.longitude, abs=1e-9)
        assert ship2_a.heading == pytest.approx(ship2_b.heading, abs=1e-9)
        assert ship2_a.sog == pytest.approx(ship2_b.sog, abs=1e-9)



# ============================================================================
# 追越场景测试
# ============================================================================

class TestOvertakingScenarioGenerator:
    """测试追越场景生成器"""
    
    def test_basic_overtaking_generation(self):
        """测试基本追越场景生成"""
        from scenario_generator import OvertakingParams
        
        generator = ScenarioGenerator()
        params = OvertakingParams(
            distance=2.0,
            speed1=12.0,  # 追越船速度更快
            speed2=8.0,   # 被追越船速度较慢
            base_latitude=30.0,
            base_longitude=120.0
        )
        
        scenario = generator.generate_overtaking_scenario(params)
        
        # 验证场景类型
        assert scenario.scenario_type == ScenarioType.OVERTAKING
        
        # 验证船舶数量
        assert len(scenario.ships) == 2
        
        ship1, ship2 = scenario.ships[0], scenario.ships[1]
        
        # 验证速度关系：追越船（船1）速度大于被追越船（船2）
        assert ship1.sog > ship2.sog, \
            f"追越船速度({ship1.sog})应大于被追越船速度({ship2.sog})"
        
        # 验证速度值
        assert ship1.sog == params.speed1
        assert ship2.sog == params.speed2
    
    def test_overtaking_position_relationship(self):
        """测试追越场景的位置关系"""
        from scenario_generator import OvertakingParams
        
        generator = ScenarioGenerator()
        params = OvertakingParams(
            distance=3.0,
            speed1=15.0,
            speed2=10.0
        )
        
        scenario = generator.generate_overtaking_scenario(params)
        ship1, ship2 = scenario.ships[0], scenario.ships[1]
        
        # 计算从被追越船（船2）看追越船（船1）的相对方位
        lon_diff = ship1.longitude - ship2.longitude
        lat_diff = ship1.latitude - ship2.latitude
        
        bearing = math.degrees(math.atan2(lon_diff, lat_diff))
        if bearing < 0:
            bearing += 360
        
        relative_bearing = bearing - ship2.heading
        if relative_bearing > 180:
            relative_bearing -= 360
        elif relative_bearing < -180:
            relative_bearing += 360
        
        # 追越船应在被追越船后方22.5度扇形区域内
        # 即相对方位在157.5-202.5度之间（或-202.5到-157.5度）
        # 简化：正后方是180度（或-180度）
        assert abs(abs(relative_bearing) - 180) <= 22.5, \
            f"追越船应在被追越船后方22.5度扇形区域内，相对方位为{relative_bearing:.2f}度"


class TestOvertakingParamsValidation:
    """测试追越场景参数验证"""
    
    def test_invalid_speed_relationship(self):
        """测试速度关系无效（追越船速度不大于被追越船）"""
        from scenario_generator import OvertakingParams
        
        with pytest.raises(ValueError, match="追越船速度.*必须大于被追越船速度"):
            OvertakingParams(
                distance=2.0,
                speed1=10.0,  # 追越船
                speed2=10.0   # 被追越船（速度相同，无效）
            )
    
    def test_invalid_speed_overtaking_slower(self):
        """测试追越船速度更慢"""
        from scenario_generator import OvertakingParams
        
        with pytest.raises(ValueError, match="追越船速度.*必须大于被追越船速度"):
            OvertakingParams(
                distance=2.0,
                speed1=8.0,   # 追越船速度更慢
                speed2=12.0   # 被追越船速度更快
            )
    
    def test_valid_speed_relationship(self):
        """测试有效的速度关系"""
        from scenario_generator import OvertakingParams
        
        params = OvertakingParams(
            distance=2.0,
            speed1=15.0,  # 追越船速度更快
            speed2=10.0   # 被追越船速度较慢
        )
        assert params.speed1 > params.speed2


# ============================================================================
# 追越场景属性测试
# ============================================================================

class TestOvertakingScenarioProperties:
    """
    追越场景生成的属性测试
    
    使用 Hypothesis 进行基于属性的测试，验证场景生成的通用正确性
    """
    
    @given(
        distance=st.floats(min_value=0.5, max_value=10.0, allow_nan=False, allow_infinity=False),
        speed2=st.floats(min_value=5.0, max_value=15.0, allow_nan=False, allow_infinity=False),
        speed_diff=st.floats(min_value=2.0, max_value=10.0, allow_nan=False, allow_infinity=False),
        base_latitude=st.floats(min_value=20.0, max_value=50.0, allow_nan=False, allow_infinity=False),
        base_longitude=st.floats(min_value=100.0, max_value=140.0, allow_nan=False, allow_infinity=False)
    )
    @settings(max_examples=100, deadline=None)
    def test_property_overtaking_speed_and_position(
        self, distance, speed2, speed_diff, base_latitude, base_longitude
    ):
        """
        **Property 3: 追越场景的速度和位置关系**
        **Validates: Requirements 1.3**
        
        Feature: maritime-collision-avoidance, Property 3: 追越场景的速度和位置关系
        
        对于任何追越场景参数，生成的追越船速度应大于被追越船，
        且追越船应位于被追越船后方22.5度扇形区域内。
        
        这个属性测试使用 Hypothesis 生成随机参数，验证所有情况下速度和位置关系都成立。
        """
        from scenario_generator import OvertakingParams
        
        # 创建场景生成器
        generator = ScenarioGenerator()
        
        # 计算追越船速度（确保大于被追越船）
        speed1 = speed2 + speed_diff
        
        # 创建参数
        params = OvertakingParams(
            distance=distance,
            speed1=speed1,  # 追越船（速度更快）
            speed2=speed2,  # 被追越船（速度较慢）
            base_latitude=base_latitude,
            base_longitude=base_longitude
        )
        
        # 生成场景
        scenario = generator.generate_overtaking_scenario(params)
        
        # 验证场景类型
        assert scenario.scenario_type == ScenarioType.OVERTAKING, \
            "场景类型应为 OVERTAKING"
        
        # 验证船舶数量
        assert len(scenario.ships) == 2, \
            "追越场景应包含2艘船舶"
        
        ship1, ship2 = scenario.ships[0], scenario.ships[1]
        
        # ========================================
        # 属性3.1：追越船速度大于被追越船速度
        # ========================================
        assert ship1.sog > ship2.sog, \
            f"追越船速度({ship1.sog:.2f}节)应大于被追越船速度({ship2.sog:.2f}节)"
        
        assert ship1.sog == pytest.approx(speed1, rel=1e-6), \
            f"追越船速度应为 {speed1} 节"
        assert ship2.sog == pytest.approx(speed2, rel=1e-6), \
            f"被追越船速度应为 {speed2} 节"
        
        # ========================================
        # 属性3.2：追越船在被追越船后方22.5度扇形区域内
        # ========================================
        # 计算从被追越船（船2）看追越船（船1）的相对方位
        lon_diff = ship1.longitude - ship2.longitude
        lat_diff = ship1.latitude - ship2.latitude
        
        # 计算方位角（从北顺时针，使用 atan2）
        bearing = math.degrees(math.atan2(lon_diff, lat_diff))
        if bearing < 0:
            bearing += 360
        
        # 计算相对方位（方位角 - 航向）
        relative_bearing = bearing - ship2.heading
        
        # 归一化到 -180 到 180 度范围
        if relative_bearing > 180:
            relative_bearing -= 360
        elif relative_bearing < -180:
            relative_bearing += 360
        
        # 验证追越船在被追越船后方22.5度扇形区域内
        # 后方22.5度扇形区域：157.5度到202.5度（或-202.5度到-157.5度）
        # 即相对方位的绝对值应在157.5到180度之间（允许小误差）
        abs_relative_bearing = abs(relative_bearing)
        
        # 允许小的浮点数误差（1度）
        tolerance = 1.0
        assert (157.5 - tolerance) <= abs_relative_bearing <= (180 + tolerance), \
            f"追越船应在被追越船后方22.5度扇形区域内，" \
            f"相对方位为 {relative_bearing:.2f} 度 " \
            f"(绝对值: {abs_relative_bearing:.2f}°, " \
            f"bearing: {bearing:.2f}°, ship2 heading: {ship2.heading:.2f}°)"
        
        # ========================================
        # 属性3.3：两船航向基本相同
        # ========================================
        # 追越场景中，两船应该沿相同或接近的航向航行
        heading_diff = abs(ship1.heading - ship2.heading)
        if heading_diff > 180:
            heading_diff = 360 - heading_diff
        
        # 航向差应该很小（允许10度以内的差异）
        assert heading_diff <= 10, \
            f"追越场景中两船航向应基本相同，航向差为 {heading_diff:.2f} 度 " \
            f"(ship1: {ship1.heading:.2f}°, ship2: {ship2.heading:.2f}°)"
        
        # ========================================
        # 额外验证：船舶距离正确
        # ========================================
        # 计算实际距离
        actual_distance_degrees = math.sqrt(
            (ship2.latitude - ship1.latitude)**2 + 
            (ship2.longitude - ship1.longitude)**2
        )
        expected_distance_degrees = distance * (1.0 / 60.0)  # 海里转度
        
        # 允许一定的误差
        assert abs(actual_distance_degrees - expected_distance_degrees) < 0.01, \
            f"船舶距离应约为 {distance} 海里，实际距离为 {actual_distance_degrees * 60:.2f} 海里"
    
    @given(
        distance=st.floats(min_value=0.5, max_value=10.0, allow_nan=False, allow_infinity=False),
        speed2=st.floats(min_value=5.0, max_value=15.0, allow_nan=False, allow_infinity=False),
        speed_diff=st.floats(min_value=2.0, max_value=10.0, allow_nan=False, allow_infinity=False)
    )
    @settings(max_examples=100, deadline=None)
    def test_property_overtaking_will_catch_up(
        self, distance, speed2, speed_diff
    ):
        """
        **Property 3 扩展：追越船会追上被追越船**
        **Validates: Requirements 1.3**
        
        Feature: maritime-collision-avoidance, Property 3: 追越场景的速度和位置关系
        
        验证生成的追越场景中，追越船最终会追上被追越船（如果不改变航向）。
        这通过验证追越船在被追越船后方且速度更快来实现。
        """
        from scenario_generator import OvertakingParams
        
        generator = ScenarioGenerator()
        
        speed1 = speed2 + speed_diff
        
        params = OvertakingParams(
            distance=distance,
            speed1=speed1,
            speed2=speed2
        )
        
        scenario = generator.generate_overtaking_scenario(params)
        ship1, ship2 = scenario.ships[0], scenario.ships[1]
        
        # 验证追越船在被追越船后方
        # 如果两船航向相同（向北），追越船应在南侧（纬度更小）
        if abs(ship1.heading - ship2.heading) < 10:
            # 根据航向判断前后关系
            if 315 <= ship2.heading or ship2.heading < 45:  # 向北
                assert ship1.latitude < ship2.latitude, \
                    "追越船应在被追越船后方（南侧）"
            elif 45 <= ship2.heading < 135:  # 向东
                assert ship1.longitude < ship2.longitude, \
                    "追越船应在被追越船后方（西侧）"
            elif 135 <= ship2.heading < 225:  # 向南
                assert ship1.latitude > ship2.latitude, \
                    "追越船应在被追越船后方（北侧）"
            elif 225 <= ship2.heading < 315:  # 向西
                assert ship1.longitude > ship2.longitude, \
                    "追越船应在被追越船后方（东侧）"
        
        # 验证速度关系
        assert ship1.sog > ship2.sog, \
            "追越船速度应大于被追越船速度"
        
        # 计算追上所需时间（简化计算）
        relative_speed = ship1.sog - ship2.sog  # 相对速度（节）
        time_to_catch_up = distance / relative_speed  # 小时
        
        # 验证追上时间是合理的（不会太长）
        assert time_to_catch_up > 0, \
            "追越船应该能够追上被追越船"
        assert time_to_catch_up < 10, \
            f"追上时间应该合理（小于10小时），实际为 {time_to_catch_up:.2f} 小时"
    
    @given(
        distance=st.floats(min_value=0.5, max_value=10.0, allow_nan=False, allow_infinity=False),
        speed2=st.floats(min_value=5.0, max_value=15.0, allow_nan=False, allow_infinity=False),
        speed_diff=st.floats(min_value=2.0, max_value=10.0, allow_nan=False, allow_infinity=False)
    )
    @settings(max_examples=100, deadline=None)
    def test_property_overtaking_scenario_consistency(
        self, distance, speed2, speed_diff
    ):
        """
        **Property 3 扩展：追越场景生成的一致性**
        **Validates: Requirements 1.3**
        
        Feature: maritime-collision-avoidance, Property 3: 追越场景的速度和位置关系
        
        验证使用相同参数多次生成场景时，结果应该一致（除了随机的场景ID）。
        """
        from scenario_generator import OvertakingParams
        
        generator = ScenarioGenerator()
        
        speed1 = speed2 + speed_diff
        
        params = OvertakingParams(
            distance=distance,
            speed1=speed1,
            speed2=speed2,
            base_latitude=30.0,
            base_longitude=120.0,
            mmsi1=123456789,
            mmsi2=987654321
        )
        
        # 生成两次场景
        scenario1 = generator.generate_overtaking_scenario(params)
        scenario2 = generator.generate_overtaking_scenario(params)
        
        # 验证船舶状态一致（除了场景ID）
        ship1_a, ship2_a = scenario1.ships[0], scenario1.ships[1]
        ship1_b, ship2_b = scenario2.ships[0], scenario2.ships[1]
        
        # 船1状态应该一致
        assert ship1_a.mmsi == ship1_b.mmsi
        assert ship1_a.latitude == pytest.approx(ship1_b.latitude, abs=1e-9)
        assert ship1_a.longitude == pytest.approx(ship1_b.longitude, abs=1e-9)
        assert ship1_a.heading == pytest.approx(ship1_b.heading, abs=1e-9)
        assert ship1_a.sog == pytest.approx(ship1_b.sog, abs=1e-9)
        
        # 船2状态应该一致
        assert ship2_a.mmsi == ship2_b.mmsi
        assert ship2_a.latitude == pytest.approx(ship2_b.latitude, abs=1e-9)
        assert ship2_a.longitude == pytest.approx(ship2_b.longitude, abs=1e-9)
        assert ship2_a.heading == pytest.approx(ship2_b.heading, abs=1e-9)
        assert ship2_a.sog == pytest.approx(ship2_b.sog, abs=1e-9)



# ============================================================================
# 多船场景测试
# ============================================================================

class TestMultiShipScenarioGenerator:
    """测试多船复杂场景生成器"""
    
    def test_basic_multi_ship_generation(self):
        """测试基本多船场景生成"""
        from scenario_generator import MultiShipParams
        
        generator = ScenarioGenerator()
        params = MultiShipParams(
            num_ships=3,
            area_size=5.0,
            base_latitude=30.0,
            base_longitude=120.0,
            min_speed=5.0,
            max_speed=20.0
        )
        
        scenario = generator.generate_multi_ship_scenario(params)
        
        # 验证场景类型
        assert scenario.scenario_type == ScenarioType.MULTI_SHIP
        
        # 验证船舶数量
        assert len(scenario.ships) == 3, \
            f"应生成3艘船舶，实际生成{len(scenario.ships)}艘"
        
        # 验证每艘船的基本属性
        for i, ship in enumerate(scenario.ships):
            assert ship.mmsi is not None, f"船{i+1}的MMSI不应为空"
            assert -90 <= ship.latitude <= 90, f"船{i+1}的纬度应在有效范围内"
            assert -180 <= ship.longitude <= 180, f"船{i+1}的经度应在有效范围内"
            assert 0 <= ship.heading < 360, f"船{i+1}的航向应在0-360度之间"
            assert params.min_speed <= ship.sog <= params.max_speed, \
                f"船{i+1}的速度应在{params.min_speed}-{params.max_speed}节之间"
    
    def test_multi_ship_with_different_counts(self):
        """测试不同数量的多船场景"""
        from scenario_generator import MultiShipParams
        
        generator = ScenarioGenerator()
        
        # 测试3艘船
        params = MultiShipParams(num_ships=3, area_size=5.0)
        scenario = generator.generate_multi_ship_scenario(params)
        assert len(scenario.ships) == 3
        
        # 测试5艘船
        params = MultiShipParams(num_ships=5, area_size=10.0)
        scenario = generator.generate_multi_ship_scenario(params)
        assert len(scenario.ships) == 5
        
        # 测试10艘船
        params = MultiShipParams(num_ships=10, area_size=15.0)
        scenario = generator.generate_multi_ship_scenario(params)
        assert len(scenario.ships) == 10
    
    def test_multi_ship_ships_within_area(self):
        """测试船舶位置在指定区域内"""
        from scenario_generator import MultiShipParams
        
        generator = ScenarioGenerator()
        params = MultiShipParams(
            num_ships=5,
            area_size=10.0,
            base_latitude=30.0,
            base_longitude=120.0
        )
        
        scenario = generator.generate_multi_ship_scenario(params)
        
        # 计算区域边界
        area_size_degrees = params.area_size * (1.0 / 60.0)
        min_lat = params.base_latitude - area_size_degrees / 2
        max_lat = params.base_latitude + area_size_degrees / 2
        min_lon = params.base_longitude - area_size_degrees / 2
        max_lon = params.base_longitude + area_size_degrees / 2
        
        # 验证所有船舶都在区域内
        for i, ship in enumerate(scenario.ships):
            assert min_lat <= ship.latitude <= max_lat, \
                f"船{i+1}的纬度{ship.latitude}应在区域内[{min_lat}, {max_lat}]"
            assert min_lon <= ship.longitude <= max_lon, \
                f"船{i+1}的经度{ship.longitude}应在区域内[{min_lon}, {max_lon}]"
    
    def test_multi_ship_minimum_distance(self):
        """测试船舶之间保持最小距离"""
        from scenario_generator import MultiShipParams
        
        generator = ScenarioGenerator()
        params = MultiShipParams(
            num_ships=5,
            area_size=10.0,
            seed=42  # 使用固定种子确保可重复性
        )
        
        scenario = generator.generate_multi_ship_scenario(params)
        
        # 最小距离（海里）
        MIN_DISTANCE = 0.3
        min_distance_degrees = MIN_DISTANCE * (1.0 / 60.0)
        
        # 检查所有船舶对之间的距离
        ships = scenario.ships
        for i in range(len(ships)):
            for j in range(i + 1, len(ships)):
                lat_diff = ships[i].latitude - ships[j].latitude
                lon_diff = ships[i].longitude - ships[j].longitude
                distance = math.sqrt(lat_diff**2 + lon_diff**2)
                
                assert distance >= min_distance_degrees, \
                    f"船{i+1}和船{j+1}之间的距离{distance * 60:.2f}海里小于最小距离{MIN_DISTANCE}海里"
    
    def test_multi_ship_unique_mmsi(self):
        """测试每艘船有唯一的MMSI"""
        from scenario_generator import MultiShipParams
        
        generator = ScenarioGenerator()
        params = MultiShipParams(num_ships=5, area_size=10.0)
        
        scenario = generator.generate_multi_ship_scenario(params)
        
        # 收集所有MMSI
        mmsi_list = [ship.mmsi for ship in scenario.ships]
        
        # 验证MMSI唯一性
        assert len(mmsi_list) == len(set(mmsi_list)), \
            "所有船舶应有唯一的MMSI"
    
    def test_multi_ship_speed_range(self):
        """测试船舶速度在指定范围内"""
        from scenario_generator import MultiShipParams
        
        generator = ScenarioGenerator()
        params = MultiShipParams(
            num_ships=5,
            area_size=10.0,
            min_speed=8.0,
            max_speed=15.0,
            seed=42
        )
        
        scenario = generator.generate_multi_ship_scenario(params)
        
        # 验证所有船舶速度在范围内
        for i, ship in enumerate(scenario.ships):
            assert params.min_speed <= ship.sog <= params.max_speed, \
                f"船{i+1}的速度{ship.sog}节应在{params.min_speed}-{params.max_speed}节之间"
    
    def test_multi_ship_heading_range(self):
        """测试船舶航向在有效范围内"""
        from scenario_generator import MultiShipParams
        
        generator = ScenarioGenerator()
        params = MultiShipParams(num_ships=5, area_size=10.0, seed=42)
        
        scenario = generator.generate_multi_ship_scenario(params)
        
        # 验证所有船舶航向在0-360度之间
        for i, ship in enumerate(scenario.ships):
            assert 0 <= ship.heading < 360, \
                f"船{i+1}的航向{ship.heading}度应在0-360度之间"
    
    def test_multi_ship_with_seed_reproducibility(self):
        """测试使用相同种子生成相同场景"""
        from scenario_generator import MultiShipParams
        
        generator = ScenarioGenerator()
        params = MultiShipParams(
            num_ships=5,
            area_size=10.0,
            base_latitude=30.0,
            base_longitude=120.0,
            min_speed=5.0,
            max_speed=20.0,
            seed=12345
        )
        
        # 生成两次场景
        scenario1 = generator.generate_multi_ship_scenario(params)
        scenario2 = generator.generate_multi_ship_scenario(params)
        
        # 验证船舶数量相同
        assert len(scenario1.ships) == len(scenario2.ships)
        
        # 验证每艘船的状态相同
        for i in range(len(scenario1.ships)):
            ship1 = scenario1.ships[i]
            ship2 = scenario2.ships[i]
            
            assert ship1.mmsi == ship2.mmsi
            assert ship1.latitude == pytest.approx(ship2.latitude, abs=1e-9)
            assert ship1.longitude == pytest.approx(ship2.longitude, abs=1e-9)
            assert ship1.heading == pytest.approx(ship2.heading, abs=1e-9)
            assert ship1.sog == pytest.approx(ship2.sog, abs=1e-9)
    
    def test_multi_ship_scenario_metadata(self):
        """测试多船场景的元数据"""
        from scenario_generator import MultiShipParams
        
        generator = ScenarioGenerator()
        params = MultiShipParams(num_ships=5, area_size=10.0)
        
        scenario = generator.generate_multi_ship_scenario(params)
        
        # 验证场景ID存在且非空
        assert scenario.scenario_id
        assert scenario.scenario_id.startswith("multi_ship_")
        
        # 验证场景描述
        assert scenario.description
        assert "多船复杂场景" in scenario.description
        assert "5艘船舶" in scenario.description
        
        # 验证成功标准
        assert 'min_distance' in scenario.success_criteria
        assert 'collision_avoided' in scenario.success_criteria
        
        # 验证持续时间
        assert scenario.duration == 600.0


class TestMultiShipParamsValidation:
    """测试多船场景参数验证"""
    
    def test_invalid_num_ships_too_few(self):
        """测试船舶数量过少（少于3艘）"""
        from scenario_generator import MultiShipParams
        
        with pytest.raises(ValueError, match="多船场景至少需要3艘船舶"):
            MultiShipParams(num_ships=2, area_size=5.0)
    
    def test_invalid_num_ships_zero(self):
        """测试船舶数量为0"""
        from scenario_generator import MultiShipParams
        
        with pytest.raises(ValueError, match="多船场景至少需要3艘船舶"):
            MultiShipParams(num_ships=0, area_size=5.0)
    
    def test_invalid_num_ships_negative(self):
        """测试船舶数量为负数"""
        from scenario_generator import MultiShipParams
        
        with pytest.raises(ValueError, match="多船场景至少需要3艘船舶"):
            MultiShipParams(num_ships=-1, area_size=5.0)
    
    def test_invalid_num_ships_too_many(self):
        """测试船舶数量过多（超过20艘）"""
        from scenario_generator import MultiShipParams
        
        with pytest.raises(ValueError, match="船舶数量不应超过20艘"):
            MultiShipParams(num_ships=21, area_size=50.0)
    
    def test_invalid_area_size_zero(self):
        """测试区域大小为0"""
        from scenario_generator import MultiShipParams
        
        with pytest.raises(ValueError, match="区域大小必须大于0"):
            MultiShipParams(num_ships=3, area_size=0.0)
    
    def test_invalid_area_size_negative(self):
        """测试区域大小为负数"""
        from scenario_generator import MultiShipParams
        
        with pytest.raises(ValueError, match="区域大小必须大于0"):
            MultiShipParams(num_ships=3, area_size=-5.0)
    
    def test_invalid_area_size_too_large(self):
        """测试区域大小过大"""
        from scenario_generator import MultiShipParams
        
        with pytest.raises(ValueError, match="区域大小不应超过50海里"):
            MultiShipParams(num_ships=3, area_size=51.0)
    
    def test_invalid_min_speed_zero(self):
        """测试最小速度为0"""
        from scenario_generator import MultiShipParams
        
        with pytest.raises(ValueError, match="最小速度必须大于0"):
            MultiShipParams(num_ships=3, area_size=5.0, min_speed=0.0)
    
    def test_invalid_min_speed_negative(self):
        """测试最小速度为负数"""
        from scenario_generator import MultiShipParams
        
        with pytest.raises(ValueError, match="最小速度必须大于0"):
            MultiShipParams(num_ships=3, area_size=5.0, min_speed=-5.0)
    
    def test_invalid_max_speed_less_than_min(self):
        """测试最大速度小于最小速度"""
        from scenario_generator import MultiShipParams
        
        with pytest.raises(ValueError, match="最大速度.*必须大于最小速度"):
            MultiShipParams(
                num_ships=3,
                area_size=5.0,
                min_speed=15.0,
                max_speed=10.0
            )
    
    def test_invalid_max_speed_equal_to_min(self):
        """测试最大速度等于最小速度"""
        from scenario_generator import MultiShipParams
        
        with pytest.raises(ValueError, match="最大速度.*必须大于最小速度"):
            MultiShipParams(
                num_ships=3,
                area_size=5.0,
                min_speed=10.0,
                max_speed=10.0
            )
    
    def test_invalid_max_speed_too_high(self):
        """测试最大速度过大"""
        from scenario_generator import MultiShipParams
        
        with pytest.raises(ValueError, match="最大速度不应超过50节"):
            MultiShipParams(
                num_ships=3,
                area_size=5.0,
                min_speed=5.0,
                max_speed=51.0
            )
    
    def test_invalid_latitude_too_low(self):
        """测试纬度过低"""
        from scenario_generator import MultiShipParams
        
        with pytest.raises(ValueError, match="纬度必须在-90到90之间"):
            MultiShipParams(
                num_ships=3,
                area_size=5.0,
                base_latitude=-91.0
            )
    
    def test_invalid_latitude_too_high(self):
        """测试纬度过高"""
        from scenario_generator import MultiShipParams
        
        with pytest.raises(ValueError, match="纬度必须在-90到90之间"):
            MultiShipParams(
                num_ships=3,
                area_size=5.0,
                base_latitude=91.0
            )
    
    def test_invalid_longitude_too_low(self):
        """测试经度过低"""
        from scenario_generator import MultiShipParams
        
        with pytest.raises(ValueError, match="经度必须在-180到180之间"):
            MultiShipParams(
                num_ships=3,
                area_size=5.0,
                base_longitude=-181.0
            )
    
    def test_invalid_longitude_too_high(self):
        """测试经度过高"""
        from scenario_generator import MultiShipParams
        
        with pytest.raises(ValueError, match="经度必须在-180到180之间"):
            MultiShipParams(
                num_ships=3,
                area_size=5.0,
                base_longitude=181.0
            )
    
    def test_valid_boundary_values(self):
        """测试边界值"""
        from scenario_generator import MultiShipParams
        
        # 最小船舶数量
        params = MultiShipParams(num_ships=3, area_size=5.0)
        assert params.num_ships == 3
        
        # 最大船舶数量
        params = MultiShipParams(num_ships=20, area_size=50.0)
        assert params.num_ships == 20
        
        # 最小区域大小
        params = MultiShipParams(num_ships=3, area_size=0.1)
        assert params.area_size == 0.1
        
        # 最大区域大小
        params = MultiShipParams(num_ships=3, area_size=50.0)
        assert params.area_size == 50.0
        
        # 边界纬度
        params = MultiShipParams(num_ships=3, area_size=5.0, base_latitude=-90.0)
        assert params.base_latitude == -90.0
        
        params = MultiShipParams(num_ships=3, area_size=5.0, base_latitude=90.0)
        assert params.base_latitude == 90.0
        
        # 边界经度
        params = MultiShipParams(num_ships=3, area_size=5.0, base_longitude=-180.0)
        assert params.base_longitude == -180.0
        
        params = MultiShipParams(num_ships=3, area_size=5.0, base_longitude=180.0)
        assert params.base_longitude == 180.0
        
        # 最大速度
        params = MultiShipParams(
            num_ships=3,
            area_size=5.0,
            min_speed=5.0,
            max_speed=50.0
        )
        assert params.max_speed == 50.0


class TestMultiShipScenarioEdgeCases:
    """测试多船场景生成器的边界情况"""
    
    def test_minimum_ships(self):
        """测试最小船舶数量（3艘）"""
        from scenario_generator import MultiShipParams
        
        generator = ScenarioGenerator()
        params = MultiShipParams(num_ships=3, area_size=5.0)
        
        scenario = generator.generate_multi_ship_scenario(params)
        assert len(scenario.ships) == 3
    
    def test_maximum_ships(self):
        """测试最大船舶数量（20艘）"""
        from scenario_generator import MultiShipParams
        
        generator = ScenarioGenerator()
        params = MultiShipParams(num_ships=20, area_size=50.0, seed=42)
        
        scenario = generator.generate_multi_ship_scenario(params)
        assert len(scenario.ships) == 20
    
    def test_small_area_few_ships(self):
        """测试小区域少量船舶"""
        from scenario_generator import MultiShipParams
        
        generator = ScenarioGenerator()
        params = MultiShipParams(num_ships=3, area_size=1.0, seed=42)
        
        scenario = generator.generate_multi_ship_scenario(params)
        assert len(scenario.ships) == 3
    
    def test_large_area_many_ships(self):
        """测试大区域大量船舶"""
        from scenario_generator import MultiShipParams
        
        generator = ScenarioGenerator()
        params = MultiShipParams(num_ships=15, area_size=40.0, seed=42)
        
        scenario = generator.generate_multi_ship_scenario(params)
        assert len(scenario.ships) == 15
    
    def test_narrow_speed_range(self):
        """测试窄速度范围"""
        from scenario_generator import MultiShipParams
        
        generator = ScenarioGenerator()
        params = MultiShipParams(
            num_ships=5,
            area_size=10.0,
            min_speed=10.0,
            max_speed=11.0,
            seed=42
        )
        
        scenario = generator.generate_multi_ship_scenario(params)
        
        # 验证所有船舶速度在窄范围内
        for ship in scenario.ships:
            assert 10.0 <= ship.sog <= 11.0
    
    def test_wide_speed_range(self):
        """测试宽速度范围"""
        from scenario_generator import MultiShipParams
        
        generator = ScenarioGenerator()
        params = MultiShipParams(
            num_ships=5,
            area_size=10.0,
            min_speed=5.0,
            max_speed=50.0,
            seed=42
        )
        
        scenario = generator.generate_multi_ship_scenario(params)
        
        # 验证所有船舶速度在宽范围内
        for ship in scenario.ships:
            assert 5.0 <= ship.sog <= 50.0
    
    def test_extreme_latitude_north(self):
        """测试极北纬度"""
        from scenario_generator import MultiShipParams
        
        generator = ScenarioGenerator()
        params = MultiShipParams(
            num_ships=3,
            area_size=5.0,
            base_latitude=85.0,
            seed=42
        )
        
        scenario = generator.generate_multi_ship_scenario(params)
        
        # 验证所有船舶位置有效
        for ship in scenario.ships:
            assert -90 <= ship.latitude <= 90
    
    def test_extreme_latitude_south(self):
        """测试极南纬度"""
        from scenario_generator import MultiShipParams
        
        generator = ScenarioGenerator()
        params = MultiShipParams(
            num_ships=3,
            area_size=5.0,
            base_latitude=-85.0,
            seed=42
        )
        
        scenario = generator.generate_multi_ship_scenario(params)
        
        # 验证所有船舶位置有效
        for ship in scenario.ships:
            assert -90 <= ship.latitude <= 90
    
    def test_date_line_crossing(self):
        """测试跨越日期变更线"""
        from scenario_generator import MultiShipParams
        
        generator = ScenarioGenerator()
        params = MultiShipParams(
            num_ships=3,
            area_size=5.0,
            base_longitude=179.0,
            seed=42
        )
        
        scenario = generator.generate_multi_ship_scenario(params)
        
        # 验证所有船舶位置有效
        for ship in scenario.ships:
            assert -180 <= ship.longitude <= 180



# ============================================================================
# 紧急场景测试
# ============================================================================

class TestEmergencyScenarioGenerator:
    """测试紧急避让场景生成器"""
    
    def test_basic_emergency_generation(self):
        """测试基本紧急场景生成"""
        from scenario_generator import EmergencyParams
        
        generator = ScenarioGenerator()
        params = EmergencyParams(
            dcpa=0.3,  # 0.3海里
            tcpa=3.0,  # 3分钟
            speed1=15.0,
            speed2=15.0,
            base_latitude=30.0,
            base_longitude=120.0
        )
        
        scenario = generator.generate_emergency_scenario(params)
        
        # 验证场景类型
        assert scenario.scenario_type == ScenarioType.EMERGENCY
        
        # 验证船舶数量
        assert len(scenario.ships) == 2
        
        ship1, ship2 = scenario.ships[0], scenario.ships[1]
        
        # 验证速度
        assert ship1.sog == params.speed1
        assert ship2.sog == params.speed2
        
        # 验证成功标准中包含DCPA和TCPA信息
        assert 'target_dcpa' in scenario.success_criteria
        assert 'target_tcpa' in scenario.success_criteria
        assert 'actual_dcpa' in scenario.success_criteria
        assert 'actual_tcpa' in scenario.success_criteria
    
    def test_emergency_dcpa_tcpa_values(self):
        """测试紧急场景的DCPA和TCPA值"""
        from scenario_generator import EmergencyParams
        
        generator = ScenarioGenerator()
        params = EmergencyParams(
            dcpa=0.4,
            tcpa=4.0,
            speed1=12.0,
            speed2=12.0
        )
        
        scenario = generator.generate_emergency_scenario(params)
        
        # 获取实际计算的DCPA和TCPA
        actual_dcpa = scenario.success_criteria['actual_dcpa']
        actual_tcpa = scenario.success_criteria['actual_tcpa']
        
        # 验证实际值接近目标值（允许一定误差）
        assert abs(actual_dcpa - params.dcpa) < 0.1, \
            f"实际DCPA({actual_dcpa:.2f})应接近目标DCPA({params.dcpa:.2f})"
        assert abs(actual_tcpa - params.tcpa) < 1.0, \
            f"实际TCPA({actual_tcpa:.2f})应接近目标TCPA({params.tcpa:.2f})"
    
    def test_emergency_with_different_speeds(self):
        """测试不同速度的紧急场景"""
        from scenario_generator import EmergencyParams
        
        generator = ScenarioGenerator()
        params = EmergencyParams(
            dcpa=0.2,
            tcpa=2.0,
            speed1=10.0,
            speed2=20.0
        )
        
        scenario = generator.generate_emergency_scenario(params)
        ship1, ship2 = scenario.ships[0], scenario.ships[1]
        
        assert ship1.sog == 10.0
        assert ship2.sog == 20.0
    
    def test_emergency_scenario_metadata(self):
        """测试紧急场景的元数据"""
        from scenario_generator import EmergencyParams
        
        generator = ScenarioGenerator()
        params = EmergencyParams(dcpa=0.3, tcpa=3.0)
        
        scenario = generator.generate_emergency_scenario(params)
        
        # 验证场景ID存在且非空
        assert scenario.scenario_id
        assert scenario.scenario_id.startswith("emergency_")
        
        # 验证场景描述
        assert scenario.description
        assert "紧急避让场景" in scenario.description
        
        # 验证成功标准
        assert 'min_distance' in scenario.success_criteria
        assert 'collision_avoided' in scenario.success_criteria


class TestEmergencyParamsValidation:
    """测试紧急场景参数验证"""
    
    def test_invalid_dcpa_negative(self):
        """测试DCPA为负数"""
        from scenario_generator import EmergencyParams
        
        with pytest.raises(ValueError, match="DCPA必须非负"):
            EmergencyParams(dcpa=-0.1, tcpa=3.0)
    
    def test_invalid_dcpa_too_large(self):
        """测试DCPA过大"""
        from scenario_generator import EmergencyParams
        
        with pytest.raises(ValueError, match="DCPA不应超过2海里"):
            EmergencyParams(dcpa=2.5, tcpa=3.0)
    
    def test_invalid_tcpa_negative(self):
        """测试TCPA为负数"""
        from scenario_generator import EmergencyParams
        
        with pytest.raises(ValueError, match="TCPA必须非负"):
            EmergencyParams(dcpa=0.3, tcpa=-1.0)
    
    def test_invalid_tcpa_too_large(self):
        """测试TCPA过大"""
        from scenario_generator import EmergencyParams
        
        with pytest.raises(ValueError, match="TCPA不应超过30分钟"):
            EmergencyParams(dcpa=0.3, tcpa=35.0)
    
    def test_invalid_speed1_zero(self):
        """测试船1速度为0"""
        from scenario_generator import EmergencyParams
        
        with pytest.raises(ValueError, match="船1速度必须大于0"):
            EmergencyParams(dcpa=0.3, tcpa=3.0, speed1=0.0)
    
    def test_invalid_speed1_negative(self):
        """测试船1速度为负数"""
        from scenario_generator import EmergencyParams
        
        with pytest.raises(ValueError, match="船1速度必须大于0"):
            EmergencyParams(dcpa=0.3, tcpa=3.0, speed1=-5.0)
    
    def test_invalid_speed2_zero(self):
        """测试船2速度为0"""
        from scenario_generator import EmergencyParams
        
        with pytest.raises(ValueError, match="船2速度必须大于0"):
            EmergencyParams(dcpa=0.3, tcpa=3.0, speed2=0.0)
    
    def test_invalid_speed_too_high(self):
        """测试速度过大"""
        from scenario_generator import EmergencyParams
        
        with pytest.raises(ValueError, match="速度不应超过50节"):
            EmergencyParams(dcpa=0.3, tcpa=3.0, speed1=51.0)
    
    def test_valid_boundary_values(self):
        """测试边界值"""
        from scenario_generator import EmergencyParams
        
        # 最小DCPA
        params = EmergencyParams(dcpa=0.0, tcpa=3.0)
        assert params.dcpa == 0.0
        
        # 最大DCPA
        params = EmergencyParams(dcpa=2.0, tcpa=3.0)
        assert params.dcpa == 2.0
        
        # 最小TCPA
        params = EmergencyParams(dcpa=0.3, tcpa=0.0)
        assert params.tcpa == 0.0
        
        # 最大TCPA
        params = EmergencyParams(dcpa=0.3, tcpa=30.0)
        assert params.tcpa == 30.0
        
        # 最大速度
        params = EmergencyParams(dcpa=0.3, tcpa=3.0, speed1=50.0, speed2=50.0)
        assert params.speed1 == 50.0
        assert params.speed2 == 50.0


# ============================================================================
# 紧急场景属性测试
# ============================================================================

class TestEmergencyScenarioProperties:
    """
    紧急避让场景生成的属性测试
    
    使用 Hypothesis 进行基于属性的测试，验证场景生成的通用正确性
    """
    
    @given(
        dcpa=st.floats(min_value=0.1, max_value=0.5, allow_nan=False, allow_infinity=False),
        tcpa=st.floats(min_value=1.0, max_value=5.0, allow_nan=False, allow_infinity=False),
        speed1=st.floats(min_value=8.0, max_value=25.0, allow_nan=False, allow_infinity=False),
        speed2=st.floats(min_value=8.0, max_value=25.0, allow_nan=False, allow_infinity=False),
        base_latitude=st.floats(min_value=20.0, max_value=50.0, allow_nan=False, allow_infinity=False),
        base_longitude=st.floats(min_value=100.0, max_value=140.0, allow_nan=False, allow_infinity=False)
    )
    @settings(max_examples=100, deadline=None)
    def test_property_emergency_risk_parameters(
        self, dcpa, tcpa, speed1, speed2, base_latitude, base_longitude
    ):
        """
        **Property 5: 危险场景的风险参数**
        **Validates: Requirements 2.1**
        
        Feature: maritime-collision-avoidance, Property 5: 危险场景的风险参数
        
        对于任何紧急避让场景配置，生成的初始状态计算出的DCPA应小于0.5海里，
        TCPA应小于5分钟。
        
        这个属性测试使用 Hypothesis 生成随机参数，验证所有情况下风险参数都满足
        紧急场景的要求。
        """
        from scenario_generator import EmergencyParams
        
        # 创建场景生成器
        generator = ScenarioGenerator()
        
        # 创建参数
        params = EmergencyParams(
            dcpa=dcpa,
            tcpa=tcpa,
            speed1=speed1,
            speed2=speed2,
            base_latitude=base_latitude,
            base_longitude=base_longitude
        )
        
        # 生成场景
        scenario = generator.generate_emergency_scenario(params)
        
        # 验证场景类型
        assert scenario.scenario_type == ScenarioType.EMERGENCY, \
            "场景类型应为 EMERGENCY"
        
        # 验证船舶数量
        assert len(scenario.ships) == 2, \
            "紧急场景应包含2艘船舶"
        
        ship1, ship2 = scenario.ships[0], scenario.ships[1]
        
        # ========================================
        # 属性5.1：验证实际计算的DCPA小于0.5海里
        # ========================================
        # 从成功标准中获取实际计算的DCPA和TCPA
        actual_dcpa = scenario.success_criteria['actual_dcpa']
        actual_tcpa = scenario.success_criteria['actual_tcpa']
        
        # 验证DCPA小于0.5海里（允许小的计算误差）
        tolerance_dcpa = 0.05  # 允许0.05海里的误差
        assert actual_dcpa < (0.5 + tolerance_dcpa), \
            f"实际DCPA({actual_dcpa:.3f}海里)应小于0.5海里 " \
            f"(目标DCPA: {dcpa:.3f}海里)"
        
        # ========================================
        # 属性5.2：验证实际计算的TCPA小于5分钟
        # ========================================
        # 验证TCPA小于5分钟（允许小的计算误差）
        tolerance_tcpa = 0.5  # 允许0.5分钟的误差
        assert actual_tcpa < (5.0 + tolerance_tcpa), \
            f"实际TCPA({actual_tcpa:.2f}分钟)应小于5分钟 " \
            f"(目标TCPA: {tcpa:.2f}分钟)"
        
        # ========================================
        # 属性5.3：验证实际DCPA接近目标DCPA
        # ========================================
        # 实际DCPA应该接近目标DCPA（允许一定的计算误差）
        # 由于反向计算的复杂性，允许较大的相对误差
        relative_error_dcpa = abs(actual_dcpa - dcpa) / max(dcpa, 0.01)
        assert relative_error_dcpa < 0.5, \
            f"实际DCPA({actual_dcpa:.3f})应接近目标DCPA({dcpa:.3f})，" \
            f"相对误差为{relative_error_dcpa:.2%}"
        
        # ========================================
        # 属性5.4：验证实际TCPA接近目标TCPA
        # ========================================
        # 实际TCPA应该接近目标TCPA（允许一定的计算误差）
        relative_error_tcpa = abs(actual_tcpa - tcpa) / max(tcpa, 0.1)
        assert relative_error_tcpa < 0.5, \
            f"实际TCPA({actual_tcpa:.2f})应接近目标TCPA({tcpa:.2f})，" \
            f"相对误差为{relative_error_tcpa:.2%}"
        
        # ========================================
        # 额外验证：船舶速度正确
        # ========================================
        assert ship1.sog == pytest.approx(speed1, rel=1e-6), \
            f"船1速度应为 {speed1} 节"
        assert ship2.sog == pytest.approx(speed2, rel=1e-6), \
            f"船2速度应为 {speed2} 节"
        
        # ========================================
        # 额外验证：两船不在同一位置
        # ========================================
        position_diff = math.sqrt(
            (ship2.latitude - ship1.latitude)**2 + 
            (ship2.longitude - ship1.longitude)**2
        )
        assert position_diff > 0.001, \
            "两船初始位置应该不同"
    
    @given(
        dcpa=st.floats(min_value=0.1, max_value=0.5, allow_nan=False, allow_infinity=False),
        tcpa=st.floats(min_value=1.0, max_value=5.0, allow_nan=False, allow_infinity=False),
        speed1=st.floats(min_value=10.0, max_value=20.0, allow_nan=False, allow_infinity=False),
        speed2=st.floats(min_value=10.0, max_value=20.0, allow_nan=False, allow_infinity=False)
    )
    @settings(max_examples=100, deadline=None)
    def test_property_emergency_collision_course(
        self, dcpa, tcpa, speed1, speed2
    ):
        """
        **Property 5 扩展：紧急场景的碰撞航线**
        **Validates: Requirements 2.1**
        
        Feature: maritime-collision-avoidance, Property 5: 危险场景的风险参数
        
        验证生成的紧急场景中，两艘船舶确实在危险的碰撞航线上。
        这通过验证DCPA和TCPA都很小来实现。
        """
        from scenario_generator import EmergencyParams
        
        generator = ScenarioGenerator()
        
        params = EmergencyParams(
            dcpa=dcpa,
            tcpa=tcpa,
            speed1=speed1,
            speed2=speed2
        )
        
        scenario = generator.generate_emergency_scenario(params)
        
        # 获取实际计算的风险参数
        actual_dcpa = scenario.success_criteria['actual_dcpa']
        actual_tcpa = scenario.success_criteria['actual_tcpa']
        
        # 验证这是一个危险场景
        # DCPA小于0.5海里表示会遇距离很近
        assert actual_dcpa < 0.55, \
            f"DCPA({actual_dcpa:.3f}海里)应小于0.5海里，表示危险相遇"
        
        # TCPA小于5分钟表示很快就会到达最近点
        assert actual_tcpa < 5.5, \
            f"TCPA({actual_tcpa:.2f}分钟)应小于5分钟，表示紧急情况"
        
        # 验证TCPA为正值（表示还未到达最近点）
        assert actual_tcpa >= 0, \
            f"TCPA应为正值，表示还未到达最近点，实际为{actual_tcpa:.2f}分钟"
    
    @given(
        dcpa=st.floats(min_value=0.1, max_value=0.5, allow_nan=False, allow_infinity=False),
        tcpa=st.floats(min_value=1.0, max_value=5.0, allow_nan=False, allow_infinity=False),
        speed1=st.floats(min_value=10.0, max_value=20.0, allow_nan=False, allow_infinity=False),
        speed2=st.floats(min_value=10.0, max_value=20.0, allow_nan=False, allow_infinity=False)
    )
    @settings(max_examples=100, deadline=None)
    def test_property_emergency_scenario_consistency(
        self, dcpa, tcpa, speed1, speed2
    ):
        """
        **Property 5 扩展：紧急场景生成的一致性**
        **Validates: Requirements 2.1**
        
        Feature: maritime-collision-avoidance, Property 5: 危险场景的风险参数
        
        验证使用相同参数多次生成场景时，结果应该一致（除了随机的场景ID）。
        """
        from scenario_generator import EmergencyParams
        
        generator = ScenarioGenerator()
        
        params = EmergencyParams(
            dcpa=dcpa,
            tcpa=tcpa,
            speed1=speed1,
            speed2=speed2,
            base_latitude=30.0,
            base_longitude=120.0,
            mmsi1=123456789,
            mmsi2=987654321
        )
        
        # 生成两次场景
        scenario1 = generator.generate_emergency_scenario(params)
        scenario2 = generator.generate_emergency_scenario(params)
        
        # 验证船舶状态一致（除了场景ID）
        ship1_a, ship2_a = scenario1.ships[0], scenario1.ships[1]
        ship1_b, ship2_b = scenario2.ships[0], scenario2.ships[1]
        
        # 船1状态应该一致
        assert ship1_a.mmsi == ship1_b.mmsi
        assert ship1_a.latitude == pytest.approx(ship1_b.latitude, abs=1e-9)
        assert ship1_a.longitude == pytest.approx(ship1_b.longitude, abs=1e-9)
        assert ship1_a.heading == pytest.approx(ship1_b.heading, abs=1e-9)
        assert ship1_a.sog == pytest.approx(ship1_b.sog, abs=1e-9)
        
        # 船2状态应该一致
        assert ship2_a.mmsi == ship2_b.mmsi
        assert ship2_a.latitude == pytest.approx(ship2_b.latitude, abs=1e-9)
        assert ship2_a.longitude == pytest.approx(ship2_b.longitude, abs=1e-9)
        assert ship2_a.heading == pytest.approx(ship2_b.heading, abs=1e-9)
        assert ship2_a.sog == pytest.approx(ship2_b.sog, abs=1e-9)
        
        # 验证实际DCPA和TCPA一致
        actual_dcpa_a = scenario1.success_criteria['actual_dcpa']
        actual_tcpa_a = scenario1.success_criteria['actual_tcpa']
        actual_dcpa_b = scenario2.success_criteria['actual_dcpa']
        actual_tcpa_b = scenario2.success_criteria['actual_tcpa']
        
        assert actual_dcpa_a == pytest.approx(actual_dcpa_b, abs=1e-9)
        assert actual_tcpa_a == pytest.approx(actual_tcpa_b, abs=1e-9)
    
    @given(
        dcpa=st.floats(min_value=0.1, max_value=0.5, allow_nan=False, allow_infinity=False),
        tcpa=st.floats(min_value=1.0, max_value=5.0, allow_nan=False, allow_infinity=False),
        speed1=st.floats(min_value=10.0, max_value=20.0, allow_nan=False, allow_infinity=False),
        speed2=st.floats(min_value=10.0, max_value=20.0, allow_nan=False, allow_infinity=False)
    )
    @settings(max_examples=100, deadline=None)
    def test_property_emergency_dcpa_tcpa_relationship(
        self, dcpa, tcpa, speed1, speed2
    ):
        """
        **Property 5 扩展：DCPA和TCPA的关系**
        **Validates: Requirements 2.1**
        
        Feature: maritime-collision-avoidance, Property 5: 危险场景的风险参数
        
        验证生成的紧急场景中，DCPA和TCPA的关系是合理的。
        较小的DCPA和TCPA表示更高的碰撞风险。
        """
        from scenario_generator import EmergencyParams
        
        generator = ScenarioGenerator()
        
        params = EmergencyParams(
            dcpa=dcpa,
            tcpa=tcpa,
            speed1=speed1,
            speed2=speed2
        )
        
        scenario = generator.generate_emergency_scenario(params)
        ship1, ship2 = scenario.ships[0], scenario.ships[1]
        
        # 获取实际计算的风险参数
        actual_dcpa = scenario.success_criteria['actual_dcpa']
        actual_tcpa = scenario.success_criteria['actual_tcpa']
        
        # 计算当前距离
        current_distance_degrees = math.sqrt(
            (ship2.latitude - ship1.latitude)**2 + 
            (ship2.longitude - ship1.longitude)**2
        )
        current_distance_nm = current_distance_degrees * 60.0  # 转换为海里
        
        # 验证当前距离大于DCPA（因为DCPA是最近会遇距离）
        # 允许小的误差，因为在某些情况下可能已经很接近最近点
        assert current_distance_nm >= actual_dcpa - 0.1, \
            f"当前距离({current_distance_nm:.3f}海里)应大于或等于DCPA({actual_dcpa:.3f}海里)"
        
        # 验证TCPA为正值（还未到达最近点）
        assert actual_tcpa >= 0, \
            f"TCPA应为正值，表示还未到达最近点"
        
        # 验证场景持续时间足够长，能够观察到避让过程
        # 持续时间应至少是TCPA的2倍（允许小的浮点数误差）
        expected_min_duration = actual_tcpa * 60 * 2
        assert scenario.duration >= expected_min_duration - 1.0, \
            f"场景持续时间({scenario.duration}秒)应至少是TCPA的2倍({expected_min_duration}秒)"
