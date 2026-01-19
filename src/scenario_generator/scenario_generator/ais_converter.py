"""
AIS模拟器配置转换器

将场景生成器的 ScenarioConfig 转换为 AIS 模拟器的 ships_config.yaml 格式
"""

import yaml
from pathlib import Path
from typing import Optional

from .models import ScenarioConfig


class AISConfigConverter:
    """AIS配置转换器
    
    将场景生成器的标准场景配置转换为AIS模拟器可用的格式
    """
    
    @staticmethod
    def scenario_to_ais_config(scenario: ScenarioConfig) -> dict:
        """将 ScenarioConfig 转换为 AIS 模拟器配置格式
        
        Args:
            scenario: 场景配置对象
            
        Returns:
            dict: AIS模拟器配置字典
        """
        simulated_ships = []
        
        for ship in scenario.ships:
            ship_config = {
                'mmsi': ship.mmsi,
                'latitude': ship.latitude,
                'longitude': ship.longitude,
                'heading': ship.heading,
                'sog': ship.sog,
                'rot': ship.rot
            }
            
            # 如果有航点，添加航点信息
            if ship.waypoints:
                ship_config['waypoints'] = ship.waypoints
            
            simulated_ships.append(ship_config)
        
        return {'simulated_ships': simulated_ships}
    
    @staticmethod
    def save_ais_config(scenario: ScenarioConfig, 
                       output_path: str,
                       add_comments: bool = True):
        """保存为AIS模拟器配置文件
        
        Args:
            scenario: 场景配置对象
            output_path: 输出文件路径
            add_comments: 是否添加注释（场景描述）
        """
        config = AISConfigConverter.scenario_to_ais_config(scenario)
        
        output_file = Path(output_path)
        output_file.parent.mkdir(parents=True, exist_ok=True)
        
        with open(output_file, 'w', encoding='utf-8') as f:
            # 添加场景描述作为注释
            if add_comments and scenario.description:
                f.write(f"# 场景: {scenario.description}\n")
                f.write(f"# 场景ID: {scenario.scenario_id}\n")
                f.write(f"# 场景类型: {scenario.scenario_type.value}\n")
                f.write(f"# 持续时间: {scenario.duration}秒\n")
                f.write("\n")
            
            # 写入YAML配置
            yaml.dump(config, f, allow_unicode=True, default_flow_style=False)
    
    @staticmethod
    def ais_config_to_scenario_ships(ais_config_path: str) -> list:
        """从AIS配置文件读取船舶信息（反向转换）
        
        Args:
            ais_config_path: AIS配置文件路径
            
        Returns:
            list: 船舶状态字典列表
        """
        with open(ais_config_path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
        
        return config.get('simulated_ships', [])


def convert_scenario_to_ais(scenario_yaml: str, 
                            output_path: Optional[str] = None) -> str:
    """便捷函数：从场景YAML生成AIS配置
    
    Args:
        scenario_yaml: 场景配置文件名（例如 'head_on_basic.yaml'）
        output_path: 输出路径，如果为None则使用默认路径
        
    Returns:
        str: 输出文件路径
    """
    from .scenario_loader import ScenarioLoader
    
    # 加载场景
    loader = ScenarioLoader()
    scenario = loader.load_from_yaml(scenario_yaml)
    
    # 确定输出路径
    if output_path is None:
        # 默认输出到 AIS 模拟器的配置目录
        output_path = 'src/ais_simulator/config/ships_config.yaml'
    
    # 转换并保存
    AISConfigConverter.save_ais_config(scenario, output_path)
    
    return output_path
