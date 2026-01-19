"""
场景生成器的单元测试

测试 ScenarioGenerator 的各种场景生成方法
"""

import pytest
import math
from scenario_generator import (
    ScenarioGenerator,
    HeadOnParams,
    ScenarioType,
    EnvironmentConfig,
)


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
