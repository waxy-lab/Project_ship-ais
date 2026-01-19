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
