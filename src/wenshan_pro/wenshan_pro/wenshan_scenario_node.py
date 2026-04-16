#!/usr/bin/env python3
"""
文山项目场景管理节点
功能：加载场景配置 -> 转换为 ships_config.yaml -> 供 AIS 模拟器使用
"""

import os
import sys
import argparse
from pathlib import Path

# 场景配置目录
SCENARIO_DIR = Path(__file__).parent.parent / 'config' / 'scenarios'
AIS_CONFIG_OUTPUT = Path(__file__).parent.parent.parent / 'ais_simulator' / 'config' / 'ships_config.yaml'

# 将工作空间 src 加入 Python 路径
ws_src = Path(__file__).parent.parent.parent
sys.path.insert(0, str(ws_src))


def list_scenarios():
    """列出所有可用场景"""
    scenarios = list(SCENARIO_DIR.glob('*.yaml'))
    print("=== 文山项目可用场景 ===")
    for i, s in enumerate(scenarios, 1):
        print(f"  {i}. {s.name}")
    return scenarios


def run_scenario(scenario_name: str, output_path: str = None, sync_install: bool = True):
    """加载场景并转换为 AIS 模拟器配置"""
    # 确保 scenario_generator 包可以被找到
    sg_path = str(ws_src / 'scenario_generator')
    if sg_path not in sys.path:
        sys.path.insert(0, sg_path)
    from scenario_generator import ScenarioLoader
    from scenario_generator.ais_converter import AISConfigConverter

    # 确定场景文件路径
    scenario_path = SCENARIO_DIR / scenario_name
    if not scenario_path.exists():
        # 尝试从全局场景目录加载
        global_scenarios = ws_src / 'scenario_generator' / 'config' / 'scenarios' / scenario_name
        if global_scenarios.exists():
            scenario_path = global_scenarios
        else:
            raise FileNotFoundError(f"找不到场景文件: {scenario_name}")

    output = output_path or str(AIS_CONFIG_OUTPUT)

    loader = ScenarioLoader()
    scenario = loader.load_from_yaml(str(scenario_path))

    print(f"已加载场景: {scenario.description}")
    print(f"船舶数量: {len(scenario.ships)}")
    for i, ship in enumerate(scenario.ships, 1):
        print(f"  船{i}: MMSI={ship.mmsi}, 位置=({ship.latitude:.4f}, {ship.longitude:.4f}), "
              f"航向={ship.heading}°, 速度={ship.sog}节")

    AISConfigConverter.save_ais_config(scenario, output)
    print(f"\n已生成 AIS 配置: {output}")

    # 同步到 install 目录（模拟器运行时实际加载的路径）
    import shutil
    ws_root = ws_src.parent
    install_config = ws_root / 'install' / 'ais_simulator' / 'share' / 'ais_simulator' / 'config' / 'ships_config.yaml'
    if sync_install and install_config.parent.exists():
        if Path(output).resolve() == install_config.resolve():
            print(f"(输出文件已在安装目录，跳过同步: {install_config})")
        else:
            shutil.copy2(output, str(install_config))
            print(f"已同步到安装目录: {install_config}")
    else:
        print(f"(安装目录不存在或已禁用同步，跳过同步: {install_config})")

    print("现在可以启动 AIS 模拟器:")
    print("  python3 -m ais_simulator.ais_sim_node --ros-args -p output_port:=/dev/pts/X")


def main():
    parser = argparse.ArgumentParser(description='文山项目场景管理工具')
    parser.add_argument('scenario', nargs='?', help='场景文件名 (如 wenshan_head_on.yaml)')
    parser.add_argument('-l', '--list', action='store_true', help='列出所有场景')
    parser.add_argument('-o', '--output', help='AIS配置输出路径')
    parser.add_argument('--no-sync-install', action='store_true', help='不自动同步到 install 目录')
    args = parser.parse_args()

    if args.list or not args.scenario:
        list_scenarios()
        return

    run_scenario(args.scenario, args.output, sync_install=not args.no_sync_install)


if __name__ == '__main__':
    main()
