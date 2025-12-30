"""
数据模型定义

定义场景配置、船舶状态、环境配置等数据结构
"""

from dataclasses import dataclass, field
from typing import List, Optional, Dict, Tuple
from enum import Enum
import math


class ScenarioType(Enum):
    """场景类型枚举"""
    HEAD_ON = "head_on"           # 对遇
    CROSSING = "crossing"         # 交叉相遇
    OVERTAKING = "overtaking"     # 追越
    MULTI_SHIP = "multi_ship"     # 多船复杂场景
    EMERGENCY = "emergency"       # 紧急场景


class WeatherCondition(Enum):
    """天气状况枚举"""
    CALM = "calm"           # 平静
    MODERATE = "moderate"   # 中等
    ROUGH = "rough"         # 恶劣


class Visibility(Enum):
    """能见度枚举"""
    GOOD = "good"           # 良好
    MODERATE = "moderate"   # 中等
    POOR = "poor"           # 不良


class WaterAreaType(Enum):
    """水域类型枚举"""
    OPEN = "open"           # 开阔水域
    RESTRICTED = "restricted"  # 受限水域
    NARROW = "narrow"       # 狭窄航道


@dataclass
class ShipState:
    """船舶状态数据模型
    
    兼容现有 ais_simulator 的字段命名：
    - sog: Speed Over Ground (对地速度)
    - rot: Rate of Turn (转向率)
    """
    mmsi: int                    # 船舶MMSI
    latitude: float              # 纬度
    longitude: float             # 经度
    heading: float               # 航向 (0-360度)
    sog: float                   # 对地速度 (节) - Speed Over Ground
    rot: float = 0.0             # 转向率 (度/分钟) - Rate of Turn
    course: Optional[float] = None      # 航迹向 (0-360度)
    navigation_status: str = "under_way_using_engine"  # 航行状态
    timestamp: float = 0.0       # 时间戳
    waypoints: Optional[List[List[float]]] = None  # 航点列表 [[lat, lon], ...]
    
    # 兼容性属性：提供 speed 和 rate_of_turn 别名
    @property
    def speed(self) -> float:
        """速度别名，兼容设计文档"""
        return self.sog
    
    @speed.setter
    def speed(self, value: float):
        self.sog = value
    
    @property
    def rate_of_turn(self) -> float:
        """转向率别名，兼容设计文档"""
        return self.rot
    
    @rate_of_turn.setter
    def rate_of_turn(self, value: float):
        self.rot = value
    
    @property
    def vx(self) -> float:
        """东向速度分量（根据设计文档）"""
        return self.sog * math.sin(math.radians(self.heading))
    
    @property
    def vy(self) -> float:
        """北向速度分量（根据设计文档）"""
        return self.sog * math.cos(math.radians(self.heading))
    
    def __post_init__(self):
        """数据验证"""
        # 验证纬度范围
        if not (-90 <= self.latitude <= 90):
            raise ValueError(f"纬度必须在-90到90之间: {self.latitude}")
        
        # 验证经度范围
        if not (-180 <= self.longitude <= 180):
            raise ValueError(f"经度必须在-180到180之间: {self.longitude}")
        
        # 验证航向范围
        if not (0 <= self.heading < 360):
            raise ValueError(f"航向必须在0到360之间: {self.heading}")
        
        # 验证速度非负
        if self.sog < 0:
            raise ValueError(f"速度不能为负数: {self.sog}")
        
        # 验证速度合理性（一般船舶速度不超过50节）
        if self.sog > 50:
            raise ValueError(f"速度超出合理范围(0-50节): {self.sog}")
        
        # 验证转向率范围（一般在-720到720度/分钟之间）
        if not (-720 <= self.rot <= 720):
            raise ValueError(f"转向率必须在-720到720度/分钟之间: {self.rot}")
        
        # 验证MMSI格式（9位数字）
        if not (100000000 <= self.mmsi <= 999999999):
            raise ValueError(f"MMSI必须是9位数字: {self.mmsi}")
        
        # 如果没有指定航迹向，默认等于航向
        if self.course is None:
            self.course = self.heading
        else:
            # 验证航迹向范围
            if not (0 <= self.course < 360):
                raise ValueError(f"航迹向必须在0到360之间: {self.course}")
        
        # 验证航点列表格式
        if self.waypoints is not None:
            if not isinstance(self.waypoints, list):
                raise ValueError("航点必须是列表格式")
            for i, waypoint in enumerate(self.waypoints):
                if not isinstance(waypoint, (list, tuple)) or len(waypoint) != 2:
                    raise ValueError(f"航点{i}格式错误，应为[lat, lon]: {waypoint}")
                lat, lon = waypoint
                if not (-90 <= lat <= 90):
                    raise ValueError(f"航点{i}纬度超出范围: {lat}")
                if not (-180 <= lon <= 180):
                    raise ValueError(f"航点{i}经度超出范围: {lon}")
    
    def to_dict(self) -> Dict:
        """转换为字典格式，兼容现有代码"""
        result = {
            'mmsi': self.mmsi,
            'latitude': self.latitude,
            'longitude': self.longitude,
            'heading': self.heading,
            'sog': self.sog,
            'rot': self.rot,
        }
        if self.waypoints:
            result['waypoints'] = self.waypoints
        return result
    
    @classmethod
    def from_dict(cls, data: Dict) -> 'ShipState':
        """从字典创建 ShipState 对象"""
        return cls(
            mmsi=data['mmsi'],
            latitude=data['latitude'],
            longitude=data['longitude'],
            heading=data['heading'],
            sog=data.get('sog', data.get('speed', 0.0)),
            rot=data.get('rot', data.get('rate_of_turn', 0.0)),
            course=data.get('course'),
            navigation_status=data.get('navigation_status', 'under_way_using_engine'),
            timestamp=data.get('timestamp', 0.0),
            waypoints=data.get('waypoints')
        )


@dataclass
class EnvironmentConfig:
    """环境配置数据模型"""
    weather_condition: WeatherCondition = WeatherCondition.CALM
    visibility: Visibility = Visibility.GOOD
    water_area_type: WaterAreaType = WaterAreaType.OPEN
    wind_speed: float = 0.0      # 风速 (m/s)
    wind_direction: float = 0.0  # 风向 (度)
    current_speed: float = 0.0   # 流速 (m/s)
    current_direction: float = 0.0  # 流向 (度)
    map_boundaries: Optional[List[Tuple[float, float]]] = None  # 地图边界
    
    def __post_init__(self):
        """数据验证"""
        # 验证风速非负且合理（一般不超过50 m/s）
        if self.wind_speed < 0:
            raise ValueError(f"风速不能为负数: {self.wind_speed}")
        if self.wind_speed > 50:
            raise ValueError(f"风速超出合理范围(0-50 m/s): {self.wind_speed}")
        
        # 验证风向范围
        if not (0 <= self.wind_direction < 360):
            raise ValueError(f"风向必须在0到360度之间: {self.wind_direction}")
        
        # 验证流速非负且合理（一般不超过5 m/s）
        if self.current_speed < 0:
            raise ValueError(f"流速不能为负数: {self.current_speed}")
        if self.current_speed > 5:
            raise ValueError(f"流速超出合理范围(0-5 m/s): {self.current_speed}")
        
        # 验证流向范围
        if not (0 <= self.current_direction < 360):
            raise ValueError(f"流向必须在0到360度之间: {self.current_direction}")
        
        # 验证地图边界格式
        if self.map_boundaries is not None:
            if not isinstance(self.map_boundaries, list):
                raise ValueError("地图边界必须是列表格式")
            if len(self.map_boundaries) < 3:
                raise ValueError("地图边界至少需要3个点构成多边形")
            for i, point in enumerate(self.map_boundaries):
                if not isinstance(point, (list, tuple)) or len(point) != 2:
                    raise ValueError(f"边界点{i}格式错误，应为(lat, lon): {point}")
                lat, lon = point
                if not (-90 <= lat <= 90):
                    raise ValueError(f"边界点{i}纬度超出范围: {lat}")
                if not (-180 <= lon <= 180):
                    raise ValueError(f"边界点{i}经度超出范围: {lon}")


@dataclass
class ScenarioConfig:
    """场景配置数据模型"""
    scenario_id: str
    scenario_type: ScenarioType
    ships: List[ShipState]
    environment: EnvironmentConfig = field(default_factory=EnvironmentConfig)
    duration: float = 600.0      # 场景持续时间（秒），默认10分钟
    success_criteria: Dict = field(default_factory=dict)  # 成功判定标准
    description: str = ""        # 场景描述
    
    def __post_init__(self):
        """数据验证"""
        # 验证场景ID非空
        if not self.scenario_id or not self.scenario_id.strip():
            raise ValueError("场景ID不能为空")
        
        # 验证船舶数量
        if len(self.ships) < 2:
            raise ValueError(f"场景至少需要2艘船舶，当前只有{len(self.ships)}艘")
        
        # 验证多船场景的船舶数量
        if self.scenario_type == ScenarioType.MULTI_SHIP and len(self.ships) < 3:
            raise ValueError(f"多船场景至少需要3艘船舶，当前只有{len(self.ships)}艘")
        
        # 验证场景持续时间
        if self.duration <= 0:
            raise ValueError(f"场景持续时间必须大于0: {self.duration}")
        if self.duration > 7200:  # 最长2小时
            raise ValueError(f"场景持续时间不应超过2小时(7200秒): {self.duration}")
        
        # 验证船舶MMSI唯一性
        mmsi_set = set()
        for ship in self.ships:
            if ship.mmsi in mmsi_set:
                raise ValueError(f"船舶MMSI重复: {ship.mmsi}")
            mmsi_set.add(ship.mmsi)
        
        # 验证船舶初始位置不重叠（距离至少10米）
        self._validate_ship_positions()
    
    def _validate_ship_positions(self):
        """验证船舶初始位置不重叠"""
        MIN_DISTANCE = 0.0001  # 约10米（纬度/经度）
        
        for i, ship1 in enumerate(self.ships):
            for j, ship2 in enumerate(self.ships[i+1:], start=i+1):
                # 计算两船之间的距离（简化的欧几里得距离）
                lat_diff = ship1.latitude - ship2.latitude
                lon_diff = ship1.longitude - ship2.longitude
                distance = math.sqrt(lat_diff**2 + lon_diff**2)
                
                if distance < MIN_DISTANCE:
                    raise ValueError(
                        f"船舶{ship1.mmsi}和{ship2.mmsi}初始位置过近 "
                        f"({distance:.6f}度，约{distance*111000:.1f}米)"
                    )
    
    def to_dict(self) -> Dict:
        """转换为字典格式"""
        return {
            'scenario_id': self.scenario_id,
            'scenario_type': self.scenario_type.value,
            'ships': [ship.to_dict() for ship in self.ships],
            'environment': {
                'weather_condition': self.environment.weather_condition.value,
                'visibility': self.environment.visibility.value,
                'water_area_type': self.environment.water_area_type.value,
                'wind_speed': self.environment.wind_speed,
                'wind_direction': self.environment.wind_direction,
                'current_speed': self.environment.current_speed,
                'current_direction': self.environment.current_direction,
                'map_boundaries': self.environment.map_boundaries,
            },
            'duration': self.duration,
            'success_criteria': self.success_criteria,
            'description': self.description,
        }
    
    @classmethod
    def from_dict(cls, data: Dict) -> 'ScenarioConfig':
        """从字典创建 ScenarioConfig 对象"""
        # 解析环境配置
        env_data = data.get('environment', {})
        environment = EnvironmentConfig(
            weather_condition=WeatherCondition(env_data.get('weather_condition', 'calm')),
            visibility=Visibility(env_data.get('visibility', 'good')),
            water_area_type=WaterAreaType(env_data.get('water_area_type', 'open')),
            wind_speed=env_data.get('wind_speed', 0.0),
            wind_direction=env_data.get('wind_direction', 0.0),
            current_speed=env_data.get('current_speed', 0.0),
            current_direction=env_data.get('current_direction', 0.0),
            map_boundaries=env_data.get('map_boundaries'),
        )
        
        # 解析船舶列表
        ships = [ShipState.from_dict(ship_data) for ship_data in data['ships']]
        
        return cls(
            scenario_id=data['scenario_id'],
            scenario_type=ScenarioType(data['scenario_type']),
            ships=ships,
            environment=environment,
            duration=data.get('duration', 600.0),
            success_criteria=data.get('success_criteria', {}),
            description=data.get('description', ''),
        )
