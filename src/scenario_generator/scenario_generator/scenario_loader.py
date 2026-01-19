"""
场景加载器 (Scenario Loader)

从YAML配置文件加载预定义场景
"""

import yaml
import os
from pathlib import Path
from typing import Optional, List

from .generator import (
    ScenarioGenerator,
    HeadOnParams,
    CrossingParams,
    OvertakingParams,
    MultiShipParams,
    EmergencyParams
)
from .models import (
    ScenarioConfig,
    ScenarioType,
    EnvironmentConfig,
    WeatherCondition,
    Visibility,
    WaterAreaType
)


class ScenarioLoader:
    """场景加载器类
    
    从YAML配置文件加载预定义场景
    """
    
    def __init__(self, scenarios_dir: Optional[str] = None):
        """初始化场景加载器
        
        Args:
            scenarios_dir: 场景配置文件目录，如果为None则使用默认目录
        """
        if scenarios_dir is None:
            # 默认目录：src/scenario_generator/config/scenarios/
            current_file = Path(__file__)
            package_dir = current_file.parent.parent
            self.scenarios_dir = package_dir / 'config' / 'scenarios'
        else:
            self.scenarios_dir = Path(scenarios_dir)
        
        if not self.scenarios_dir.exists():
            raise FileNotFoundError(f"场景目录不存在: {self.scenarios_dir}")
    
    def list_scenarios(self) -> List[str]:
        """列出所有可用的场景文件
        
        Returns:
            场景文件名列表
        """
        yaml_files = list(self.scenarios_dir.glob('*.yaml'))
        return [f.name for f in yaml_files]
    
    def load_from_yaml(self, filename: str) -> ScenarioConfig:
        """从YAML文件加载场景配置
        
        Args:
            filename: 场景文件名（例如 'head_on_basic.yaml'）
            
        Returns:
            ScenarioConfig: 场景配置对象
            
        Raises:
            FileNotFoundError: 文件不存在
            ValueError: 配置格式错误
        """
        filepath = self.scenarios_dir / filename
        
        if not filepath.exists():
            raise FileNotFoundError(f"场景文件不存在: {filepath}")
        
        # 读取YAML文件
        with open(filepath, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
        
        # 解析场景类型
        scenario_type_str = config.get('scenario_type')
        if not scenario_type_str:
            raise ValueError("配置文件缺少 'scenario_type' 字段")
        
        # 解析环境配置
        env_config = config.get('environment', {})
        environment = EnvironmentConfig(
            weather_condition=WeatherCondition(env_config.get('weather_condition', 'calm')),
            visibility=Visibility(env_config.get('visibility', 'good')),
            water_area_type=WaterAreaType(env_config.get('water_area_type', 'open')),
            wind_speed=env_config.get('wind_speed', 0.0),
            wind_direction=env_config.get('wind_direction', 0.0),
            current_speed=env_config.get('current_speed', 0.0),
            current_direction=env_config.get('current_direction', 0.0),
            map_boundaries=env_config.get('map_boundaries')
        )
        
        # 创建场景生成器
        generator = ScenarioGenerator(environment=environment)
        
        # 根据场景类型生成场景
        params_dict = config.get('parameters', {})
        
        if scenario_type_str == 'head_on':
            params = HeadOnParams(
                distance=params_dict['distance'],
                speed1=params_dict['speed1'],
                speed2=params_dict['speed2'],
                base_latitude=params_dict.get('base_latitude', 30.0),
                base_longitude=params_dict.get('base_longitude', 120.0),
                mmsi1=params_dict.get('mmsi1', 123456789),
                mmsi2=params_dict.get('mmsi2', 987654321)
            )
            scenario = generator.generate_head_on_scenario(params)
        
        elif scenario_type_str == 'crossing':
            params = CrossingParams(
                distance=params_dict['distance'],
                speed1=params_dict['speed1'],
                speed2=params_dict['speed2'],
                crossing_angle=params_dict['crossing_angle'],
                base_latitude=params_dict.get('base_latitude', 30.0),
                base_longitude=params_dict.get('base_longitude', 120.0),
                mmsi1=params_dict.get('mmsi1', 123456789),
                mmsi2=params_dict.get('mmsi2', 987654321)
            )
            scenario = generator.generate_crossing_scenario(params)
        
        elif scenario_type_str == 'overtaking':
            params = OvertakingParams(
                distance=params_dict['distance'],
                speed1=params_dict['speed1'],
                speed2=params_dict['speed2'],
                base_latitude=params_dict.get('base_latitude', 30.0),
                base_longitude=params_dict.get('base_longitude', 120.0),
                mmsi1=params_dict.get('mmsi1', 123456789),
                mmsi2=params_dict.get('mmsi2', 987654321)
            )
            scenario = generator.generate_overtaking_scenario(params)
        
        elif scenario_type_str == 'multi_ship':
            params = MultiShipParams(
                num_ships=params_dict['num_ships'],
                area_size=params_dict['area_size'],
                base_latitude=params_dict.get('base_latitude', 30.0),
                base_longitude=params_dict.get('base_longitude', 120.0)
            )
            scenario = generator.generate_multi_ship_scenario(params)
        
        elif scenario_type_str == 'emergency':
            params = EmergencyParams(
                dcpa=params_dict['dcpa'],
                tcpa=params_dict['tcpa'],
                base_latitude=params_dict.get('base_latitude', 30.0),
                base_longitude=params_dict.get('base_longitude', 120.0),
                mmsi1=params_dict.get('mmsi1', 123456789),
                mmsi2=params_dict.get('mmsi2', 987654321)
            )
            scenario = generator.generate_emergency_scenario(params)
        
        else:
            raise ValueError(f"不支持的场景类型: {scenario_type_str}")
        
        # 更新场景描述和持续时间
        if 'description' in config:
            scenario.description = config['description']
        if 'duration' in config:
            scenario.duration = config['duration']
        if 'success_criteria' in config:
            scenario.success_criteria = config['success_criteria']
        
        return scenario
    
    def save_to_yaml(self, scenario: ScenarioConfig, filename: str):
        """将场景配置保存为YAML文件
        
        Args:
            scenario: 场景配置对象
            filename: 保存的文件名
        """
        filepath = self.scenarios_dir / filename
        
        # 转换为字典格式
        config_dict = scenario.to_dict()
        
        # 保存为YAML
        with open(filepath, 'w', encoding='utf-8') as f:
            yaml.dump(config_dict, f, allow_unicode=True, default_flow_style=False)
