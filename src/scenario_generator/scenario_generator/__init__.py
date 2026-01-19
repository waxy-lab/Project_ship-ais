"""
场景生成器模块 (Scenario Generator Module)

该模块负责生成各种船舶相遇场景，包括：
- 对遇场景 (Head-on)
- 交叉相遇场景 (Crossing)
- 追越场景 (Overtaking)
- 多船复杂场景 (Multi-ship)
- 极端场景 (Emergency)
"""

__version__ = "0.1.0"

from .models import (
    ScenarioType,
    WeatherCondition,
    Visibility,
    WaterAreaType,
    ShipState,
    EnvironmentConfig,
    ScenarioConfig
)

from .generator import (
    ScenarioGenerator,
    HeadOnParams,
    CrossingParams,
    OvertakingParams,
    MultiShipParams,
    EmergencyParams
)

from .scenario_loader import ScenarioLoader

__all__ = [
    'ScenarioType',
    'WeatherCondition',
    'Visibility',
    'WaterAreaType',
    'ShipState',
    'EnvironmentConfig',
    'ScenarioConfig',
    'ScenarioGenerator',
    'HeadOnParams',
    'CrossingParams',
    'OvertakingParams',
    'MultiShipParams',
    'EmergencyParams',
    'ScenarioLoader'
]
