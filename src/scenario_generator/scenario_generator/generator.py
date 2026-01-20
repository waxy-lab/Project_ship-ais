"""
场景生成器 (Scenario Generator)

根据配置参数生成标准化的船舶相遇场景
"""

from dataclasses import dataclass
from typing import Optional
import math
import uuid

from .models import (
    ScenarioConfig,
    ScenarioType,
    ShipState,
    EnvironmentConfig
)


@dataclass
class HeadOnParams:
    """对遇场景参数"""
    distance: float              # 两船初始距离（海里）
    speed1: float                # 船1速度（节）
    speed2: float                # 船2速度（节）
    base_latitude: float = 30.0  # 基准纬度
    base_longitude: float = 120.0  # 基准经度
    mmsi1: int = 123456789       # 船1 MMSI
    mmsi2: int = 987654321       # 船2 MMSI
    
    def __post_init__(self):
        """参数验证"""
        if self.distance <= 0:
            raise ValueError(f"距离必须大于0: {self.distance}")
        if self.distance > 20:
            raise ValueError(f"距离不应超过20海里: {self.distance}")
        if self.speed1 <= 0:
            raise ValueError(f"船1速度必须大于0: {self.speed1}")
        if self.speed2 <= 0:
            raise ValueError(f"船2速度必须大于0: {self.speed2}")
        if self.speed1 > 50 or self.speed2 > 50:
            raise ValueError(f"速度不应超过50节")
        if not (-90 <= self.base_latitude <= 90):
            raise ValueError(f"纬度必须在-90到90之间: {self.base_latitude}")
        if not (-180 <= self.base_longitude <= 180):
            raise ValueError(f"经度必须在-180到180之间: {self.base_longitude}")


@dataclass
class CrossingParams:
    """交叉相遇场景参数"""
    distance: float              # 两船初始距离（海里）
    speed1: float                # 船1速度（节）
    speed2: float                # 船2速度（节）
    crossing_angle: float        # 交叉角度（度，5-112.5 或 -112.5到-5）
    base_latitude: float = 30.0  # 基准纬度
    base_longitude: float = 120.0  # 基准经度
    mmsi1: int = 123456789       # 船1 MMSI
    mmsi2: int = 987654321       # 船2 MMSI
    
    def __post_init__(self):
        """参数验证"""
        if self.distance <= 0:
            raise ValueError(f"距离必须大于0: {self.distance}")
        if self.distance > 20:
            raise ValueError(f"距离不应超过20海里: {self.distance}")
        if self.speed1 <= 0:
            raise ValueError(f"船1速度必须大于0: {self.speed1}")
        if self.speed2 <= 0:
            raise ValueError(f"船2速度必须大于0: {self.speed2}")
        if self.speed1 > 50 or self.speed2 > 50:
            raise ValueError(f"速度不应超过50节")
        if not (-90 <= self.base_latitude <= 90):
            raise ValueError(f"纬度必须在-90到90之间: {self.base_latitude}")
        if not (-180 <= self.base_longitude <= 180):
            raise ValueError(f"经度必须在-180到180之间: {self.base_longitude}")
        
        # 验证交叉角度范围：5-112.5 或 -112.5到-5
        if not ((5 <= self.crossing_angle <= 112.5) or (-112.5 <= self.crossing_angle <= -5)):
            raise ValueError(
                f"交叉角度必须在5-112.5度或-112.5到-5度之间: {self.crossing_angle}"
            )


@dataclass
class OvertakingParams:
    """追越场景参数"""
    distance: float              # 两船初始距离（海里）
    speed1: float                # 追越船速度（节）
    speed2: float                # 被追越船速度（节）
    base_latitude: float = 30.0  # 基准纬度
    base_longitude: float = 120.0  # 基准经度
    mmsi1: int = 123456789       # 追越船 MMSI
    mmsi2: int = 987654321       # 被追越船 MMSI


@dataclass
class MultiShipParams:
    """多船场景参数"""
    num_ships: int               # 船舶数量
    area_size: float             # 区域大小（海里）
    base_latitude: float = 30.0  # 基准纬度
    base_longitude: float = 120.0  # 基准经度


@dataclass
class EmergencyParams:
    """紧急场景参数"""
    dcpa: float                  # 目标DCPA（海里）
    tcpa: float                  # 目标TCPA（分钟）
    base_latitude: float = 30.0  # 基准纬度
    base_longitude: float = 120.0  # 基准经度
    mmsi1: int = 123456789       # 船1 MMSI
    mmsi2: int = 987654321       # 船2 MMSI


class ScenarioGenerator:
    """场景生成器类
    
    根据配置参数生成标准化的船舶相遇场景
    """
    
    # 常量定义
    NAUTICAL_MILE_TO_DEGREE = 1.0 / 60.0  # 1海里 ≈ 1/60度（纬度）
    
    def __init__(self, environment: Optional[EnvironmentConfig] = None):
        """初始化场景生成器
        
        Args:
            environment: 环境配置，如果为None则使用默认配置
        """
        self.environment = environment or EnvironmentConfig()
    
    def generate_head_on_scenario(self, params: HeadOnParams) -> ScenarioConfig:
        """生成对遇场景
        
        对遇场景定义：两艘船舶相向航行，航向差接近180度
        
        根据设计文档和需求：
        - 航向差应在170-190度之间
        - 相对方位应在-10到10度之间
        - 两船在同一直线上相向航行
        
        Args:
            params: 对遇场景参数
            
        Returns:
            ScenarioConfig: 生成的场景配置
            
        Raises:
            ValueError: 参数无效时抛出
        """
        # 计算距离对应的经纬度偏移
        # 简化处理：假设在相对较小的区域内，经度和纬度的度数近似相等
        distance_in_degrees = params.distance * self.NAUTICAL_MILE_TO_DEGREE
        
        # 船1位置：在基准位置的西侧，航向向东（90度）
        ship1_lon = params.base_longitude - distance_in_degrees / 2
        ship1_lat = params.base_latitude
        ship1_heading = 90.0  # 向东
        
        # 船2位置：在基准位置的东侧，航向向西（270度）
        ship2_lon = params.base_longitude + distance_in_degrees / 2
        ship2_lat = params.base_latitude
        ship2_heading = 270.0  # 向西
        
        # 创建船舶状态
        ship1 = ShipState(
            mmsi=params.mmsi1,
            latitude=ship1_lat,
            longitude=ship1_lon,
            heading=ship1_heading,
            sog=params.speed1,
            rot=0.0,
            navigation_status="under_way_using_engine"
        )
        
        ship2 = ShipState(
            mmsi=params.mmsi2,
            latitude=ship2_lat,
            longitude=ship2_lon,
            heading=ship2_heading,
            sog=params.speed2,
            rot=0.0,
            navigation_status="under_way_using_engine"
        )
        
        # 生成场景ID
        scenario_id = f"head_on_{uuid.uuid4().hex[:8]}"
        
        # 创建场景配置
        scenario = ScenarioConfig(
            scenario_id=scenario_id,
            scenario_type=ScenarioType.HEAD_ON,
            ships=[ship1, ship2],
            environment=self.environment,
            duration=600.0,  # 默认10分钟
            success_criteria={
                'min_distance': 0.5,  # 最小距离0.5海里
                'collision_avoided': True
            },
            description=f"对遇场景：两船相距{params.distance}海里，"
                       f"船1速度{params.speed1}节向东，"
                       f"船2速度{params.speed2}节向西"
        )
        
        return scenario
    
    def generate_crossing_scenario(self, params: CrossingParams) -> ScenarioConfig:
        """生成交叉相遇场景
        
        交叉相遇场景定义：两艘船舶以一定角度交叉航行
        
        根据设计文档和需求：
        - 相对方位应在 5-112.5 度或 -112.5 到 -5 度之间
        - 两船的航线会在某点相交
        - crossing_angle 是从船1看船2的相对方位
        
        实现策略（简化方法）：
        - 船1位于基准位置，航向向北（0度）
        - 船2位于船1的相对方位crossing_angle方向，距离为params.distance
        - 计算两船航线的交叉点
        - 调整两船位置，使它们距离交叉点相等
        
        Args:
            params: 交叉相遇场景参数
            
        Returns:
            ScenarioConfig: 生成的场景配置
            
        Raises:
            ValueError: 参数无效时抛出
        """
        # 计算距离对应的经纬度偏移
        distance_in_degrees = params.distance * self.NAUTICAL_MILE_TO_DEGREE
        
        # 船1的初始设置
        ship1_lat = params.base_latitude
        ship1_lon = params.base_longitude
        ship1_heading = 0.0  # 向北
        
        # 从船1看船2的绝对方位
        bearing_to_ship2 = (ship1_heading + params.crossing_angle) % 360
        
        # 船2的初始位置：在船1的相对方位crossing_angle方向，距离为distance
        ship2_lat = ship1_lat + distance_in_degrees * math.cos(math.radians(bearing_to_ship2))
        ship2_lon = ship1_lon + distance_in_degrees * math.sin(math.radians(bearing_to_ship2))
        
        # 计算两船航线的交叉点
        # 船1沿航向0度（向北）前进
        # 船2需要朝向一个方向，使得两条航线相交
        
        # 简化：让船2的航向指向船1前方的某个点
        # 这样可以确保两船会相遇
        
        # 计算交叉点：设交叉点在船1前方distance距离处
        crossing_lat = ship1_lat + distance_in_degrees
        crossing_lon = ship1_lon
        
        # 计算船2的航向：从船2指向交叉点
        lat_diff = crossing_lat - ship2_lat
        lon_diff = crossing_lon - ship2_lon
        ship2_heading = math.degrees(math.atan2(lon_diff, lat_diff))
        if ship2_heading < 0:
            ship2_heading += 360
        
        # 创建船舶状态
        ship1 = ShipState(
            mmsi=params.mmsi1,
            latitude=ship1_lat,
            longitude=ship1_lon,
            heading=ship1_heading,
            sog=params.speed1,
            rot=0.0,
            navigation_status="under_way_using_engine"
        )
        
        ship2 = ShipState(
            mmsi=params.mmsi2,
            latitude=ship2_lat,
            longitude=ship2_lon,
            heading=ship2_heading,
            sog=params.speed2,
            rot=0.0,
            navigation_status="under_way_using_engine"
        )
        
        # 生成场景ID
        scenario_id = f"crossing_{uuid.uuid4().hex[:8]}"
        
        # 创建场景配置
        scenario = ScenarioConfig(
            scenario_id=scenario_id,
            scenario_type=ScenarioType.CROSSING,
            ships=[ship1, ship2],
            environment=self.environment,
            duration=600.0,  # 默认10分钟
            success_criteria={
                'min_distance': 0.5,  # 最小距离0.5海里
                'collision_avoided': True
            },
            description=f"交叉相遇场景：两船初始相距{params.distance}海里，"
                       f"交叉角度{params.crossing_angle}度，"
                       f"船1速度{params.speed1}节航向{ship1_heading:.1f}度，"
                       f"船2速度{params.speed2}节航向{ship2_heading:.1f}度"
        )
        
        return scenario
    
    def generate_overtaking_scenario(self, params: OvertakingParams) -> ScenarioConfig:
        """生成追越场景
        
        Args:
            params: 追越场景参数
            
        Returns:
            ScenarioConfig: 生成的场景配置
        """
        # TODO: 实现追越场景生成
        raise NotImplementedError("追越场景生成尚未实现")
    
    def generate_multi_ship_scenario(self, params: MultiShipParams) -> ScenarioConfig:
        """生成多船复杂场景
        
        Args:
            params: 多船场景参数
            
        Returns:
            ScenarioConfig: 生成的场景配置
        """
        # TODO: 实现多船场景生成
        raise NotImplementedError("多船场景生成尚未实现")
    
    def generate_emergency_scenario(self, params: EmergencyParams) -> ScenarioConfig:
        """生成紧急场景
        
        Args:
            params: 紧急场景参数
            
        Returns:
            ScenarioConfig: 生成的场景配置
        """
        # TODO: 实现紧急场景生成
        raise NotImplementedError("紧急场景生成尚未实现")
