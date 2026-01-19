"""
场景配置数据模型的单元测试

测试 ShipState, EnvironmentConfig, ScenarioConfig 的数据验证逻辑
"""

import pytest
import math
from scenario_generator.models import (
    ShipState,
    EnvironmentConfig,
    ScenarioConfig,
    ScenarioType,
    WeatherCondition,
    Visibility,
    WaterAreaType,
)


class TestShipState:
    """测试 ShipState 数据模型"""
    
    def test_valid_ship_state(self):
        """测试创建有效的船舶状态"""
        ship = ShipState(
            mmsi=123456789,
            latitude=30.0,
            longitude=120.0,
            heading=90.0,
            sog=10.0,
            rot=5.0
        )
        
        assert ship.mmsi == 123456789
        assert ship.latitude == 30.0
        assert ship.longitude == 120.0
        assert ship.heading == 90.0
        assert ship.sog == 10.0
        assert ship.rot == 5.0
        assert ship.course == 90.0  # 默认等于航向
    
    def test_speed_alias(self):
        """测试 speed 属性别名"""
        ship = ShipState(
            mmsi=123456789,
            latitude=30.0,
            longitude=120.0,
            heading=90.0,
            sog=10.0
        )
        
        assert ship.speed == 10.0
        ship.speed = 15.0
        assert ship.sog == 15.0
    
    def test_velocity_components(self):
        """测试速度分量计算"""
        ship = ShipState(
            mmsi=123456789,
            latitude=30.0,
            longitude=120.0,
            heading=90.0,  # 正东
            sog=10.0
        )
        
        # 正东方向：vx应该接近10，vy应该接近0
        assert abs(ship.vx - 10.0) < 0.01
        assert abs(ship.vy) < 0.01
        
        # 正北方向
        ship.heading = 0.0
        assert abs(ship.vx) < 0.01
        assert abs(ship.vy - 10.0) < 0.01
    
    def test_invalid_latitude_too_low(self):
        """测试纬度过低"""
        with pytest.raises(ValueError, match="纬度必须在-90到90之间"):
            ShipState(
                mmsi=123456789,
                latitude=-91.0,
                longitude=120.0,
                heading=90.0,
                sog=10.0
            )
    
    def test_invalid_latitude_too_high(self):
        """测试纬度过高"""
        with pytest.raises(ValueError, match="纬度必须在-90到90之间"):
            ShipState(
                mmsi=123456789,
                latitude=91.0,
                longitude=120.0,
                heading=90.0,
                sog=10.0
            )
    
    def test_invalid_longitude_too_low(self):
        """测试经度过低"""
        with pytest.raises(ValueError, match="经度必须在-180到180之间"):
            ShipState(
                mmsi=123456789,
                latitude=30.0,
                longitude=-181.0,
                heading=90.0,
                sog=10.0
            )
    
    def test_invalid_longitude_too_high(self):
        """测试经度过高"""
        with pytest.raises(ValueError, match="经度必须在-180到180之间"):
            ShipState(
                mmsi=123456789,
                latitude=30.0,
                longitude=181.0,
                heading=90.0,
                sog=10.0
            )
    
    def test_invalid_heading_negative(self):
        """测试航向为负数"""
        with pytest.raises(ValueError, match="航向必须在0到360之间"):
            ShipState(
                mmsi=123456789,
                latitude=30.0,
                longitude=120.0,
                heading=-1.0,
                sog=10.0
            )
    
    def test_invalid_heading_too_high(self):
        """测试航向过大"""
        with pytest.raises(ValueError, match="航向必须在0到360之间"):
            ShipState(
                mmsi=123456789,
                latitude=30.0,
                longitude=120.0,
                heading=360.0,
                sog=10.0
            )
    
    def test_invalid_speed_negative(self):
        """测试速度为负数"""
        with pytest.raises(ValueError, match="速度不能为负数"):
            ShipState(
                mmsi=123456789,
                latitude=30.0,
                longitude=120.0,
                heading=90.0,
                sog=-1.0
            )
    
    def test_invalid_speed_too_high(self):
        """测试速度过大"""
        with pytest.raises(ValueError, match="速度超出合理范围"):
            ShipState(
                mmsi=123456789,
                latitude=30.0,
                longitude=120.0,
                heading=90.0,
                sog=51.0
            )
    
    def test_invalid_rot_too_low(self):
        """测试转向率过低"""
        with pytest.raises(ValueError, match="转向率必须在-720到720度/分钟之间"):
            ShipState(
                mmsi=123456789,
                latitude=30.0,
                longitude=120.0,
                heading=90.0,
                sog=10.0,
                rot=-721.0
            )
    
    def test_invalid_rot_too_high(self):
        """测试转向率过高"""
        with pytest.raises(ValueError, match="转向率必须在-720到720度/分钟之间"):
            ShipState(
                mmsi=123456789,
                latitude=30.0,
                longitude=120.0,
                heading=90.0,
                sog=10.0,
                rot=721.0
            )
    
    def test_invalid_mmsi_too_short(self):
        """测试MMSI位数不足"""
        with pytest.raises(ValueError, match="MMSI必须是9位数字"):
            ShipState(
                mmsi=12345678,  # 8位
                latitude=30.0,
                longitude=120.0,
                heading=90.0,
                sog=10.0
            )
    
    def test_invalid_mmsi_too_long(self):
        """测试MMSI位数过多"""
        with pytest.raises(ValueError, match="MMSI必须是9位数字"):
            ShipState(
                mmsi=1234567890,  # 10位
                latitude=30.0,
                longitude=120.0,
                heading=90.0,
                sog=10.0
            )
    
    def test_invalid_course(self):
        """测试无效的航迹向"""
        with pytest.raises(ValueError, match="航迹向必须在0到360之间"):
            ShipState(
                mmsi=123456789,
                latitude=30.0,
                longitude=120.0,
                heading=90.0,
                sog=10.0,
                course=370.0
            )
    
    def test_invalid_waypoints_format(self):
        """测试无效的航点格式"""
        with pytest.raises(ValueError, match="航点.*格式错误"):
            ShipState(
                mmsi=123456789,
                latitude=30.0,
                longitude=120.0,
                heading=90.0,
                sog=10.0,
                waypoints=[[30.0]]  # 缺少经度
            )
    
    def test_invalid_waypoints_latitude(self):
        """测试航点纬度超出范围"""
        with pytest.raises(ValueError, match="航点.*纬度超出范围"):
            ShipState(
                mmsi=123456789,
                latitude=30.0,
                longitude=120.0,
                heading=90.0,
                sog=10.0,
                waypoints=[[91.0, 120.0]]
            )
    
    def test_valid_waypoints(self):
        """测试有效的航点列表"""
        ship = ShipState(
            mmsi=123456789,
            latitude=30.0,
            longitude=120.0,
            heading=90.0,
            sog=10.0,
            waypoints=[[30.5, 120.5], [31.0, 121.0]]
        )
        
        assert len(ship.waypoints) == 2
        assert ship.waypoints[0] == [30.5, 120.5]
    
    def test_to_dict(self):
        """测试转换为字典"""
        ship = ShipState(
            mmsi=123456789,
            latitude=30.0,
            longitude=120.0,
            heading=90.0,
            sog=10.0,
            waypoints=[[30.5, 120.5]]
        )
        
        data = ship.to_dict()
        assert data['mmsi'] == 123456789
        assert data['latitude'] == 30.0
        assert data['waypoints'] == [[30.5, 120.5]]
    
    def test_from_dict(self):
        """测试从字典创建"""
        data = {
            'mmsi': 123456789,
            'latitude': 30.0,
            'longitude': 120.0,
            'heading': 90.0,
            'sog': 10.0,
        }
        
        ship = ShipState.from_dict(data)
        assert ship.mmsi == 123456789
        assert ship.latitude == 30.0


class TestEnvironmentConfig:
    """测试 EnvironmentConfig 数据模型"""
    
    def test_valid_environment_config(self):
        """测试创建有效的环境配置"""
        env = EnvironmentConfig(
            weather_condition=WeatherCondition.CALM,
            visibility=Visibility.GOOD,
            water_area_type=WaterAreaType.OPEN,
            wind_speed=5.0,
            wind_direction=90.0,
            current_speed=1.0,
            current_direction=180.0
        )
        
        assert env.weather_condition == WeatherCondition.CALM
        assert env.wind_speed == 5.0
        assert env.current_speed == 1.0
    
    def test_default_values(self):
        """测试默认值"""
        env = EnvironmentConfig()
        
        assert env.weather_condition == WeatherCondition.CALM
        assert env.visibility == Visibility.GOOD
        assert env.wind_speed == 0.0
        assert env.current_speed == 0.0
    
    def test_invalid_wind_speed_negative(self):
        """测试风速为负数"""
        with pytest.raises(ValueError, match="风速不能为负数"):
            EnvironmentConfig(wind_speed=-1.0)
    
    def test_invalid_wind_speed_too_high(self):
        """测试风速过大"""
        with pytest.raises(ValueError, match="风速超出合理范围"):
            EnvironmentConfig(wind_speed=51.0)
    
    def test_invalid_wind_direction(self):
        """测试无效的风向"""
        with pytest.raises(ValueError, match="风向必须在0到360度之间"):
            EnvironmentConfig(wind_direction=370.0)
    
    def test_invalid_current_speed_negative(self):
        """测试流速为负数"""
        with pytest.raises(ValueError, match="流速不能为负数"):
            EnvironmentConfig(current_speed=-1.0)
    
    def test_invalid_current_speed_too_high(self):
        """测试流速过大"""
        with pytest.raises(ValueError, match="流速超出合理范围"):
            EnvironmentConfig(current_speed=6.0)
    
    def test_invalid_current_direction(self):
        """测试无效的流向"""
        with pytest.raises(ValueError, match="流向必须在0到360度之间"):
            EnvironmentConfig(current_direction=-10.0)
    
    def test_invalid_map_boundaries_too_few_points(self):
        """测试地图边界点数不足"""
        with pytest.raises(ValueError, match="地图边界至少需要3个点"):
            EnvironmentConfig(map_boundaries=[(30.0, 120.0), (31.0, 121.0)])
    
    def test_invalid_map_boundaries_format(self):
        """测试地图边界格式错误"""
        with pytest.raises(ValueError, match="边界点.*格式错误"):
            EnvironmentConfig(map_boundaries=[(30.0,), (31.0, 121.0), (32.0, 122.0)])
    
    def test_invalid_map_boundaries_latitude(self):
        """测试地图边界纬度超出范围"""
        with pytest.raises(ValueError, match="边界点.*纬度超出范围"):
            EnvironmentConfig(
                map_boundaries=[(91.0, 120.0), (31.0, 121.0), (32.0, 122.0)]
            )
    
    def test_valid_map_boundaries(self):
        """测试有效的地图边界"""
        env = EnvironmentConfig(
            map_boundaries=[(30.0, 120.0), (31.0, 121.0), (32.0, 122.0)]
        )
        
        assert len(env.map_boundaries) == 3


class TestScenarioConfig:
    """测试 ScenarioConfig 数据模型"""
    
    def create_valid_ships(self, count=2):
        """创建有效的船舶列表"""
        ships = []
        for i in range(count):
            ships.append(ShipState(
                mmsi=100000000 + i,
                latitude=30.0 + i * 0.01,
                longitude=120.0 + i * 0.01,
                heading=90.0,
                sog=10.0
            ))
        return ships
    
    def test_valid_scenario_config(self):
        """测试创建有效的场景配置"""
        ships = self.create_valid_ships(2)
        scenario = ScenarioConfig(
            scenario_id="test_001",
            scenario_type=ScenarioType.HEAD_ON,
            ships=ships,
            duration=600.0
        )
        
        assert scenario.scenario_id == "test_001"
        assert scenario.scenario_type == ScenarioType.HEAD_ON
        assert len(scenario.ships) == 2
        assert scenario.duration == 600.0
    
    def test_invalid_empty_scenario_id(self):
        """测试空的场景ID"""
        ships = self.create_valid_ships(2)
        
        with pytest.raises(ValueError, match="场景ID不能为空"):
            ScenarioConfig(
                scenario_id="",
                scenario_type=ScenarioType.HEAD_ON,
                ships=ships
            )
    
    def test_invalid_whitespace_scenario_id(self):
        """测试只有空格的场景ID"""
        ships = self.create_valid_ships(2)
        
        with pytest.raises(ValueError, match="场景ID不能为空"):
            ScenarioConfig(
                scenario_id="   ",
                scenario_type=ScenarioType.HEAD_ON,
                ships=ships
            )
    
    def test_invalid_too_few_ships(self):
        """测试船舶数量不足"""
        ships = self.create_valid_ships(1)
        
        with pytest.raises(ValueError, match="场景至少需要2艘船舶"):
            ScenarioConfig(
                scenario_id="test_001",
                scenario_type=ScenarioType.HEAD_ON,
                ships=ships
            )
    
    def test_invalid_multi_ship_too_few(self):
        """测试多船场景船舶数量不足"""
        ships = self.create_valid_ships(2)
        
        with pytest.raises(ValueError, match="多船场景至少需要3艘船舶"):
            ScenarioConfig(
                scenario_id="test_001",
                scenario_type=ScenarioType.MULTI_SHIP,
                ships=ships
            )
    
    def test_valid_multi_ship_scenario(self):
        """测试有效的多船场景"""
        ships = self.create_valid_ships(3)
        scenario = ScenarioConfig(
            scenario_id="test_001",
            scenario_type=ScenarioType.MULTI_SHIP,
            ships=ships
        )
        
        assert len(scenario.ships) == 3
    
    def test_invalid_duration_zero(self):
        """测试持续时间为0"""
        ships = self.create_valid_ships(2)
        
        with pytest.raises(ValueError, match="场景持续时间必须大于0"):
            ScenarioConfig(
                scenario_id="test_001",
                scenario_type=ScenarioType.HEAD_ON,
                ships=ships,
                duration=0.0
            )
    
    def test_invalid_duration_negative(self):
        """测试持续时间为负数"""
        ships = self.create_valid_ships(2)
        
        with pytest.raises(ValueError, match="场景持续时间必须大于0"):
            ScenarioConfig(
                scenario_id="test_001",
                scenario_type=ScenarioType.HEAD_ON,
                ships=ships,
                duration=-100.0
            )
    
    def test_invalid_duration_too_long(self):
        """测试持续时间过长"""
        ships = self.create_valid_ships(2)
        
        with pytest.raises(ValueError, match="场景持续时间不应超过2小时"):
            ScenarioConfig(
                scenario_id="test_001",
                scenario_type=ScenarioType.HEAD_ON,
                ships=ships,
                duration=7201.0
            )
    
    def test_invalid_duplicate_mmsi(self):
        """测试重复的MMSI"""
        ships = [
            ShipState(
                mmsi=123456789,
                latitude=30.0,
                longitude=120.0,
                heading=90.0,
                sog=10.0
            ),
            ShipState(
                mmsi=123456789,  # 重复的MMSI
                latitude=30.01,
                longitude=120.01,
                heading=270.0,
                sog=10.0
            )
        ]
        
        with pytest.raises(ValueError, match="船舶MMSI重复"):
            ScenarioConfig(
                scenario_id="test_001",
                scenario_type=ScenarioType.HEAD_ON,
                ships=ships
            )
    
    def test_invalid_overlapping_positions(self):
        """测试船舶位置重叠"""
        ships = [
            ShipState(
                mmsi=123456789,
                latitude=30.0,
                longitude=120.0,
                heading=90.0,
                sog=10.0
            ),
            ShipState(
                mmsi=987654321,
                latitude=30.0,  # 相同位置
                longitude=120.0,
                heading=270.0,
                sog=10.0
            )
        ]
        
        with pytest.raises(ValueError, match="初始位置过近"):
            ScenarioConfig(
                scenario_id="test_001",
                scenario_type=ScenarioType.HEAD_ON,
                ships=ships
            )
    
    def test_to_dict(self):
        """测试转换为字典"""
        ships = self.create_valid_ships(2)
        scenario = ScenarioConfig(
            scenario_id="test_001",
            scenario_type=ScenarioType.HEAD_ON,
            ships=ships,
            description="测试场景"
        )
        
        data = scenario.to_dict()
        assert data['scenario_id'] == "test_001"
        assert data['scenario_type'] == "head_on"
        assert len(data['ships']) == 2
        assert data['description'] == "测试场景"
    
    def test_from_dict(self):
        """测试从字典创建"""
        data = {
            'scenario_id': 'test_001',
            'scenario_type': 'head_on',
            'ships': [
                {
                    'mmsi': 123456789,
                    'latitude': 30.0,
                    'longitude': 120.0,
                    'heading': 90.0,
                    'sog': 10.0,
                },
                {
                    'mmsi': 987654321,
                    'latitude': 30.01,
                    'longitude': 120.01,
                    'heading': 270.0,
                    'sog': 10.0,
                }
            ],
            'environment': {
                'weather_condition': 'calm',
                'visibility': 'good',
                'water_area_type': 'open',
            },
            'duration': 600.0,
        }
        
        scenario = ScenarioConfig.from_dict(data)
        assert scenario.scenario_id == 'test_001'
        assert scenario.scenario_type == ScenarioType.HEAD_ON
        assert len(scenario.ships) == 2


class TestBoundaryConditions:
    """测试边界条件"""
    
    def test_latitude_boundary_min(self):
        """测试纬度最小边界值"""
        ship = ShipState(
            mmsi=123456789,
            latitude=-90.0,  # 最小值
            longitude=120.0,
            heading=90.0,
            sog=10.0
        )
        assert ship.latitude == -90.0
    
    def test_latitude_boundary_max(self):
        """测试纬度最大边界值"""
        ship = ShipState(
            mmsi=123456789,
            latitude=90.0,  # 最大值
            longitude=120.0,
            heading=90.0,
            sog=10.0
        )
        assert ship.latitude == 90.0
    
    def test_longitude_boundary_min(self):
        """测试经度最小边界值"""
        ship = ShipState(
            mmsi=123456789,
            latitude=30.0,
            longitude=-180.0,  # 最小值
            heading=90.0,
            sog=10.0
        )
        assert ship.longitude == -180.0
    
    def test_longitude_boundary_max(self):
        """测试经度最大边界值"""
        ship = ShipState(
            mmsi=123456789,
            latitude=30.0,
            longitude=180.0,  # 最大值
            heading=90.0,
            sog=10.0
        )
        assert ship.longitude == 180.0
    
    def test_heading_boundary_min(self):
        """测试航向最小边界值"""
        ship = ShipState(
            mmsi=123456789,
            latitude=30.0,
            longitude=120.0,
            heading=0.0,  # 最小值
            sog=10.0
        )
        assert ship.heading == 0.0
    
    def test_heading_boundary_max(self):
        """测试航向最大边界值"""
        ship = ShipState(
            mmsi=123456789,
            latitude=30.0,
            longitude=120.0,
            heading=359.9,  # 接近最大值
            sog=10.0
        )
        assert ship.heading == 359.9
    
    def test_speed_boundary_min(self):
        """测试速度最小边界值"""
        ship = ShipState(
            mmsi=123456789,
            latitude=30.0,
            longitude=120.0,
            heading=90.0,
            sog=0.0  # 最小值（静止）
        )
        assert ship.sog == 0.0
    
    def test_speed_boundary_max(self):
        """测试速度最大边界值"""
        ship = ShipState(
            mmsi=123456789,
            latitude=30.0,
            longitude=120.0,
            heading=90.0,
            sog=50.0  # 最大值
        )
        assert ship.sog == 50.0
    
    def test_rot_boundary_min(self):
        """测试转向率最小边界值"""
        ship = ShipState(
            mmsi=123456789,
            latitude=30.0,
            longitude=120.0,
            heading=90.0,
            sog=10.0,
            rot=-720.0  # 最小值
        )
        assert ship.rot == -720.0
    
    def test_rot_boundary_max(self):
        """测试转向率最大边界值"""
        ship = ShipState(
            mmsi=123456789,
            latitude=30.0,
            longitude=120.0,
            heading=90.0,
            sog=10.0,
            rot=720.0  # 最大值
        )
        assert ship.rot == 720.0
    
    def test_mmsi_boundary_min(self):
        """测试MMSI最小边界值"""
        ship = ShipState(
            mmsi=100000000,  # 最小9位数
            latitude=30.0,
            longitude=120.0,
            heading=90.0,
            sog=10.0
        )
        assert ship.mmsi == 100000000
    
    def test_mmsi_boundary_max(self):
        """测试MMSI最大边界值"""
        ship = ShipState(
            mmsi=999999999,  # 最大9位数
            latitude=30.0,
            longitude=120.0,
            heading=90.0,
            sog=10.0
        )
        assert ship.mmsi == 999999999
    
    def test_duration_boundary_min(self):
        """测试持续时间最小边界值"""
        ships = [
            ShipState(
                mmsi=123456789,
                latitude=30.0,
                longitude=120.0,
                heading=90.0,
                sog=10.0
            ),
            ShipState(
                mmsi=987654321,
                latitude=30.01,
                longitude=120.01,
                heading=270.0,
                sog=10.0
            )
        ]
        
        scenario = ScenarioConfig(
            scenario_id="test_001",
            scenario_type=ScenarioType.HEAD_ON,
            ships=ships,
            duration=0.1  # 接近最小值
        )
        assert scenario.duration == 0.1
    
    def test_duration_boundary_max(self):
        """测试持续时间最大边界值"""
        ships = [
            ShipState(
                mmsi=123456789,
                latitude=30.0,
                longitude=120.0,
                heading=90.0,
                sog=10.0
            ),
            ShipState(
                mmsi=987654321,
                latitude=30.01,
                longitude=120.01,
                heading=270.0,
                sog=10.0
            )
        ]
        
        scenario = ScenarioConfig(
            scenario_id="test_001",
            scenario_type=ScenarioType.HEAD_ON,
            ships=ships,
            duration=7200.0  # 最大值（2小时）
        )
        assert scenario.duration == 7200.0
