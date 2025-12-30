"""
数据模型定义

定义场景配置、船舶状态、环境配置等数据结构
"""

from dataclasses import dataclass, field
from typing import List, Optional, Dict, Tuple
from enum import Enum


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
    
    def __post_init__(self):
        """数据验证"""
        if not (0 <= self.latitude <= 90 or -90 <= self.latitude < 0):
            raise ValueError(f"纬度必须在-90到90之间: {self.latitude}")
        if not (-180 <= self.longitude <= 180):
            raise ValueError(f"经度必须在-180到180之间: {self.longitude}")
        if not (0 <= self.heading < 360):
            raise ValueError(f"航向必须在0到360之间: {self.heading}")
        if self.sog < 0:
            raise ValueError(f"速度不能为负数: {self.sog}")
        if self.course is None:
            self.course = self.heading
    
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
        if len(self.ships) < 2:
            raise ValueError("场景至少需要2艘船舶")
        if self.duration <= 0:
            raise ValueError("场景持续时间必须大于0")
