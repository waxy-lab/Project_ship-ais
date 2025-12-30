"""
配置文件加载器

支持从YAML文件加载场景配置
"""

import yaml
from pathlib import Path
from typing import Dict, Any, Optional
from .models import (
    ScenarioConfig, 
    ScenarioType, 
    ShipState, 
    EnvironmentConfig,
    WeatherCondition,
    Visibility,
    WaterAreaType
)


class ConfigLoader:
    """配置文件加载器"""
    
    @staticmethod
    def load_yaml(file_path: str) -> Dict[str, Any]:
        """加载YAML配置文件
        
        Args:
            file_path: YAML文件路径
            
        Returns:
            配置字典
            
        Raises:
            FileNotFoundError: 文件不存在
            yaml.YAMLError: YAML解析错误
        """
        path = Path(file_path)
        if not path.exists():
            raise FileNotFoundError(f"配置文件不存在: {file_path}")
        
        with open(path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
        
        return config
    
    @staticmethod
    def load_scenario_config(file_path: str) -> ScenarioConfig:
        """从YAML文件加载场景配置
        
        Args:
            file_path: 场景配置文件路径
            
        Returns:
            ScenarioConfig对象
        """
        config_dict = ConfigLoader.load_yaml(file_path)
        
        # 解析场景类型
        scenario_type = ScenarioType(config_dict.get('scenario_type', 'head_on'))
        
        # 解析船舶列表
        ships = []
        for ship_data in config_dict.get('ships', []):
            ship = ShipState.from_dict(ship_data)
            ships.append(ship)
        
        # 解析环境配置
        env_data = config_dict.get('environment', {})
        environment = EnvironmentConfig(
            weather_condition=WeatherCondition(env_data.get('weather_condition', 'calm')),
            visibility=Visibility(env_data.get('visibility', 'good')),
            water_area_type=WaterAreaType(env_data.get('water_area_type', 'open')),
            wind_speed=env_data.get('wind_speed', 0.0),
            wind_direction=env_data.get('wind_direction', 0.0),
            current_speed=env_data.get('current_speed', 0.0),
            current_direction=env_data.get('current_direction', 0.0),
            map_boundaries=env_data.get('map_boundaries')
        )
        
        # 创建场景配置
        scenario = ScenarioConfig(
            scenario_id=config_dict.get('scenario_id', 'unknown'),
            scenario_type=scenario_type,
            ships=ships,
            environment=environment,
            duration=config_dict.get('duration', 600.0),
            success_criteria=config_dict.get('success_criteria', {}),
            description=config_dict.get('description', '')
        )
        
        return scenario
    
    @staticmethod
    def save_scenario_config(scenario: ScenarioConfig, file_path: str):
        """保存场景配置到YAML文件
        
        Args:
            scenario: ScenarioConfig对象
            file_path: 保存路径
        """
        config_dict = {
            'scenario_id': scenario.scenario_id,
            'scenario_type': scenario.scenario_type.value,
            'description': scenario.description,
            'duration': scenario.duration,
            'ships': [ship.to_dict() for ship in scenario.ships],
            'environment': {
                'weather_condition': scenario.environment.weather_condition.value,
                'visibility': scenario.environment.visibility.value,
                'water_area_type': scenario.environment.water_area_type.value,
                'wind_speed': scenario.environment.wind_speed,
                'wind_direction': scenario.environment.wind_direction,
                'current_speed': scenario.environment.current_speed,
                'current_direction': scenario.environment.current_direction,
                'map_boundaries': scenario.environment.map_boundaries
            },
            'success_criteria': scenario.success_criteria
        }
        
        path = Path(file_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        
        with open(path, 'w', encoding='utf-8') as f:
            yaml.dump(config_dict, f, default_flow_style=False, allow_unicode=True)
