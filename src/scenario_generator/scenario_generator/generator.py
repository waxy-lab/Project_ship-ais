"""
场景生成器 (Scenario Generator)

根据配置参数生成标准化的船舶相遇场景
"""

from dataclasses import dataclass
from typing import Optional
import math
import uuid
import random

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
    
    def __post_init__(self):
        """参数验证"""
        if self.distance <= 0:
            raise ValueError(f"距离必须大于0: {self.distance}")
        if self.distance > 20:
            raise ValueError(f"距离不应超过20海里: {self.distance}")
        if self.speed1 <= 0:
            raise ValueError(f"追越船速度必须大于0: {self.speed1}")
        if self.speed2 <= 0:
            raise ValueError(f"被追越船速度必须大于0: {self.speed2}")
        if self.speed1 <= self.speed2:
            raise ValueError(f"追越船速度({self.speed1})必须大于被追越船速度({self.speed2})")
        if self.speed1 > 50 or self.speed2 > 50:
            raise ValueError(f"速度不应超过50节")
        if not (-90 <= self.base_latitude <= 90):
            raise ValueError(f"纬度必须在-90到90之间: {self.base_latitude}")
        if not (-180 <= self.base_longitude <= 180):
            raise ValueError(f"经度必须在-180到180之间: {self.base_longitude}")


@dataclass
class MultiShipParams:
    """多船场景参数"""
    num_ships: int               # 船舶数量
    area_size: float             # 区域大小（海里）
    base_latitude: float = 30.0  # 基准纬度
    base_longitude: float = 120.0  # 基准经度
    min_speed: float = 5.0       # 最小速度（节）
    max_speed: float = 20.0      # 最大速度（节）
    seed: Optional[int] = None   # 随机种子（用于可重复性）
    
    def __post_init__(self):
        """参数验证"""
        if self.num_ships < 3:
            raise ValueError(f"多船场景至少需要3艘船舶: {self.num_ships}")
        if self.num_ships > 20:
            raise ValueError(f"船舶数量不应超过20艘: {self.num_ships}")
        if self.area_size <= 0:
            raise ValueError(f"区域大小必须大于0: {self.area_size}")
        if self.area_size > 50:
            raise ValueError(f"区域大小不应超过50海里: {self.area_size}")
        if not (-90 <= self.base_latitude <= 90):
            raise ValueError(f"纬度必须在-90到90之间: {self.base_latitude}")
        if not (-180 <= self.base_longitude <= 180):
            raise ValueError(f"经度必须在-180到180之间: {self.base_longitude}")
        if self.min_speed <= 0:
            raise ValueError(f"最小速度必须大于0: {self.min_speed}")
        if self.max_speed <= self.min_speed:
            raise ValueError(f"最大速度({self.max_speed})必须大于最小速度({self.min_speed})")
        if self.max_speed > 50:
            raise ValueError(f"最大速度不应超过50节: {self.max_speed}")


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
        
        追越场景定义：一艘船从后方追赶另一艘船
        
        根据设计文档和需求（COLREGS Rule 13）：
        - 追越船必须在被追越船后方22.5度扇形区域内
        - 追越船速度必须大于被追越船速度
        - 两船航向基本相同
        
        实现策略：
        - 被追越船（船2）位于基准位置，航向向北（0度）
        - 追越船（船1）位于被追越船后方，相对方位在后方22.5度扇形区域内
        - 具体位置：在被追越船正后方（相对方位180度），距离为params.distance
        - 两船航向相同，但追越船速度更快
        
        Args:
            params: 追越场景参数
            
        Returns:
            ScenarioConfig: 生成的场景配置
            
        Raises:
            ValueError: 参数无效时抛出
        """
        # 计算距离对应的经纬度偏移
        distance_in_degrees = params.distance * self.NAUTICAL_MILE_TO_DEGREE
        
        # 被追越船（船2）的设置：位于基准位置，航向向北
        ship2_lat = params.base_latitude
        ship2_lon = params.base_longitude
        ship2_heading = 0.0  # 向北
        
        # 追越船（船1）的设置：位于被追越船正后方
        # 相对方位180度（正后方）在22.5度扇形区域内（157.5-202.5度）
        # 为了简化，将追越船放在被追越船正后方
        ship1_lat = ship2_lat - distance_in_degrees  # 在南侧（后方）
        ship1_lon = ship2_lon
        ship1_heading = ship2_heading  # 航向相同
        
        # 创建船舶状态
        # 船1是追越船（速度更快）
        ship1 = ShipState(
            mmsi=params.mmsi1,
            latitude=ship1_lat,
            longitude=ship1_lon,
            heading=ship1_heading,
            sog=params.speed1,
            rot=0.0,
            navigation_status="under_way_using_engine"
        )
        
        # 船2是被追越船（速度较慢）
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
        scenario_id = f"overtaking_{uuid.uuid4().hex[:8]}"
        
        # 创建场景配置
        scenario = ScenarioConfig(
            scenario_id=scenario_id,
            scenario_type=ScenarioType.OVERTAKING,
            ships=[ship1, ship2],
            environment=self.environment,
            duration=600.0,  # 默认10分钟
            success_criteria={
                'min_distance': 0.5,  # 最小距离0.5海里
                'collision_avoided': True
            },
            description=f"追越场景：追越船（船1）在被追越船（船2）后方{params.distance}海里，"
                       f"追越船速度{params.speed1}节，被追越船速度{params.speed2}节，"
                       f"两船航向均为{ship1_heading:.1f}度"
        )
        
        return scenario
    
    def generate_multi_ship_scenario(self, params: MultiShipParams) -> ScenarioConfig:
        """生成多船复杂场景
        
        多船场景定义：3艘及以上船舶在指定区域内随机分布
        
        根据需求 1.4：
        - 支持3艘及以上船舶
        - 船舶在指定区域内随机分布
        - 每艘船有随机的航向和速度
        
        实现策略：
        - 在以base_latitude, base_longitude为中心的正方形区域内随机生成船舶位置
        - 区域大小由area_size参数指定（海里）
        - 每艘船的航向随机（0-360度）
        - 每艘船的速度在min_speed和max_speed之间随机
        - 确保船舶之间有最小安全距离，避免初始位置重叠
        
        Args:
            params: 多船场景参数
            
        Returns:
            ScenarioConfig: 生成的场景配置
            
        Raises:
            ValueError: 参数无效时抛出
        """
        # 设置随机种子（如果提供）以确保可重复性
        if params.seed is not None:
            random.seed(params.seed)
        
        # 计算区域大小对应的经纬度范围
        # area_size是海里，转换为度
        area_size_degrees = params.area_size * self.NAUTICAL_MILE_TO_DEGREE
        
        # 最小船舶间距（海里），避免初始位置过近
        MIN_SHIP_DISTANCE = 0.3  # 0.3海里
        min_distance_degrees = MIN_SHIP_DISTANCE * self.NAUTICAL_MILE_TO_DEGREE
        
        ships = []
        base_mmsi = 100000000  # 起始MMSI
        
        # 生成船舶
        max_attempts = 1000  # 最大尝试次数，避免无限循环
        attempts = 0
        
        for i in range(params.num_ships):
            # 尝试找到一个不与现有船舶重叠的位置
            position_found = False
            
            while not position_found and attempts < max_attempts:
                attempts += 1
                
                # 在区域内随机生成位置
                # 使用均匀分布在正方形区域内
                lat_offset = random.uniform(-area_size_degrees/2, area_size_degrees/2)
                lon_offset = random.uniform(-area_size_degrees/2, area_size_degrees/2)
                
                ship_lat = params.base_latitude + lat_offset
                ship_lon = params.base_longitude + lon_offset
                
                # 检查与现有船舶的距离
                too_close = False
                for existing_ship in ships:
                    lat_diff = ship_lat - existing_ship.latitude
                    lon_diff = ship_lon - existing_ship.longitude
                    distance = math.sqrt(lat_diff**2 + lon_diff**2)
                    
                    if distance < min_distance_degrees:
                        too_close = True
                        break
                
                if not too_close:
                    position_found = True
                    
                    # 随机生成航向（0-360度）
                    heading = random.uniform(0, 360)
                    
                    # 随机生成速度（在min_speed和max_speed之间）
                    speed = random.uniform(params.min_speed, params.max_speed)
                    
                    # 创建船舶状态
                    ship = ShipState(
                        mmsi=base_mmsi + i,
                        latitude=ship_lat,
                        longitude=ship_lon,
                        heading=heading,
                        sog=speed,
                        rot=0.0,
                        navigation_status="under_way_using_engine"
                    )
                    
                    ships.append(ship)
            
            if not position_found:
                raise ValueError(
                    f"无法在{max_attempts}次尝试内为第{i+1}艘船找到合适位置。"
                    f"请尝试增加区域大小或减少船舶数量。"
                )
        
        # 生成场景ID
        scenario_id = f"multi_ship_{uuid.uuid4().hex[:8]}"
        
        # 创建场景描述
        description = (
            f"多船复杂场景：{params.num_ships}艘船舶在"
            f"{params.area_size}海里区域内随机分布，"
            f"速度范围{params.min_speed}-{params.max_speed}节"
        )
        
        # 创建场景配置
        scenario = ScenarioConfig(
            scenario_id=scenario_id,
            scenario_type=ScenarioType.MULTI_SHIP,
            ships=ships,
            environment=self.environment,
            duration=600.0,  # 默认10分钟
            success_criteria={
                'min_distance': 0.5,  # 最小距离0.5海里
                'collision_avoided': True
            },
            description=description
        )
        
        return scenario
    
    def generate_emergency_scenario(self, params: EmergencyParams) -> ScenarioConfig:
        """生成紧急场景
        
        Args:
            params: 紧急场景参数
            
        Returns:
            ScenarioConfig: 生成的场景配置
        """
        # TODO: 实现紧急场景生成
        raise NotImplementedError("紧急场景生成尚未实现")
