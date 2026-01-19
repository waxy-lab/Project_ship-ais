#!/usr/bin/env python3
"""
场景转AIS配置转换工具

将场景生成器的场景配置转换为AIS模拟器可用的配置文件
"""

import argparse
import sys
from pathlib import Path

from src.scenario_generator.scenario_generator import (
    ScenarioLoader,
    AISConfigConverter
)


def main():
    parser = argparse.ArgumentParser(
        description='将场景配置转换为AIS模拟器配置',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 转换基础对遇场景
  python convert_scenario_to_ais.py head_on_basic.yaml
  
  # 转换并指定输出路径
  python convert_scenario_to_ais.py head_on_basic.yaml -o custom_ships.yaml
  
  # 列出所有可用场景
  python convert_scenario_to_ais.py --list
        """
    )
    
    parser.add_argument(
        'scenario_file',
        nargs='?',
        help='场景配置文件名（例如 head_on_basic.yaml）'
    )
    
    parser.add_argument(
        '-o', '--output',
        default='src/ais_simulator/config/ships_config.yaml',
        help='输出文件路径（默认: src/ais_simulator/config/ships_config.yaml）'
    )
    
    parser.add_argument(
        '-l', '--list',
        action='store_true',
        help='列出所有可用的场景'
    )
    
    parser.add_argument(
        '--no-comments',
        action='store_true',
        help='不添加场景描述注释'
    )
    
    args = parser.parse_args()
    
    # 创建加载器
    loader = ScenarioLoader()
    
    # 列出场景
    if args.list:
        print("可用场景列表：")
        print("=" * 60)
        scenarios = loader.list_scenarios()
        for i, scenario_file in enumerate(scenarios, 1):
            print(f"  {i}. {scenario_file}")
        print("=" * 60)
        print(f"共 {len(scenarios)} 个场景")
        return 0
    
    # 检查是否提供了场景文件
    if not args.scenario_file:
        parser.print_help()
        return 1
    
    try:
        # 加载场景
        print(f"正在加载场景: {args.scenario_file}")
        scenario = loader.load_from_yaml(args.scenario_file)
        
        print(f"  场景ID: {scenario.scenario_id}")
        print(f"  场景类型: {scenario.scenario_type.value}")
        print(f"  船舶数量: {len(scenario.ships)}")
        print(f"  描述: {scenario.description}")
        
        # 转换并保存
        print(f"\n正在转换为AIS配置...")
        AISConfigConverter.save_ais_config(
            scenario,
            args.output,
            add_comments=not args.no_comments
        )
        
        print(f"✓ 成功保存到: {args.output}")
        
        # 显示船舶信息
        print(f"\n船舶配置:")
        for i, ship in enumerate(scenario.ships, 1):
            print(f"  船{i}: MMSI={ship.mmsi}, "
                  f"位置=({ship.latitude:.5f}, {ship.longitude:.5f}), "
                  f"航向={ship.heading}°, 速度={ship.sog}节")
            if ship.waypoints:
                print(f"       航点数量: {len(ship.waypoints)}")
        
        print("\n转换完成！现在可以启动AIS模拟器：")
        print(f"  ros2 run ais_simulator ais_sim_node")
        
        return 0
        
    except FileNotFoundError as e:
        print(f"错误: 场景文件不存在 - {e}", file=sys.stderr)
        return 1
    except Exception as e:
        print(f"错误: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        return 1


if __name__ == '__main__':
    sys.exit(main())
