#!/usr/bin/env python3
"""
CRI（碰撞风险指数）计算演示脚本

演示如何使用风险评估模块计算碰撞风险指数
"""

import sys
import os

# 添加路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src/scenario_generator'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src/collision_avoidance'))

from scenario_generator.models import ShipState
from collision_avoidance.risk_assessment import (
    assess_collision_risk,
    calculate_cri,
    RiskLevel
)


def print_separator():
    """打印分隔线"""
    print("=" * 80)


def demo_high_risk_scenario():
    """演示高风险场景"""
    print_separator()
    print("场景 1: 高风险对遇场景")
    print_separator()
    
    # 两船相向航行，距离很近
    own_ship = ShipState(
        mmsi=123456789,
        latitude=0.0,
        longitude=0.0,
        heading=90.0,  # 向东
        sog=10.0
    )
    
    target_ship = ShipState(
        mmsi=987654321,
        latitude=0.0,
        longitude=0.03,  # 约1.8海里
        heading=270.0,  # 向西
        sog=12.0
    )
    
    # 评估风险
    risk = assess_collision_risk(own_ship, target_ship)
    
    print(f"本船: MMSI={own_ship.mmsi}, 位置=({own_ship.latitude:.4f}, {own_ship.longitude:.4f})")
    print(f"      航向={own_ship.heading:.1f}°, 速度={own_ship.sog:.1f}节")
    print(f"目标船: MMSI={target_ship.mmsi}, 位置=({target_ship.latitude:.4f}, {target_ship.longitude:.4f})")
    print(f"        航向={target_ship.heading:.1f}°, 速度={target_ship.sog:.1f}节")
    print()
    print("风险评估结果:")
    print(f"  DCPA (最近会遇距离): {risk.dcpa:.2f} 海里")
    print(f"  TCPA (到达最近点时间): {risk.tcpa:.2f} 分钟")
    print(f"  CRI (碰撞风险指数): {risk.cri:.3f}")
    print(f"  风险等级: {risk.risk_level.value.upper()}")
    print(f"  相对方位: {risk.relative_bearing:.1f}°")
    print(f"  当前距离: {risk.distance:.2f} 海里")
    print(f"  速度比: {risk.speed_ratio:.2f}")
    print()


def demo_medium_risk_scenario():
    """演示中等风险场景"""
    print_separator()
    print("场景 2: 中等风险交叉场景")
    print_separator()
    
    # 两船交叉相遇
    own_ship = ShipState(
        mmsi=123456789,
        latitude=0.0,
        longitude=0.0,
        heading=0.0,  # 向北
        sog=10.0
    )
    
    target_ship = ShipState(
        mmsi=987654321,
        latitude=0.02,  # 北侧约1.2海里
        longitude=0.05,  # 东侧约3海里
        heading=270.0,  # 向西
        sog=10.0
    )
    
    # 评估风险
    risk = assess_collision_risk(own_ship, target_ship)
    
    print(f"本船: MMSI={own_ship.mmsi}, 位置=({own_ship.latitude:.4f}, {own_ship.longitude:.4f})")
    print(f"      航向={own_ship.heading:.1f}°, 速度={own_ship.sog:.1f}节")
    print(f"目标船: MMSI={target_ship.mmsi}, 位置=({target_ship.latitude:.4f}, {target_ship.longitude:.4f})")
    print(f"        航向={target_ship.heading:.1f}°, 速度={target_ship.sog:.1f}节")
    print()
    print("风险评估结果:")
    print(f"  DCPA (最近会遇距离): {risk.dcpa:.2f} 海里")
    print(f"  TCPA (到达最近点时间): {risk.tcpa:.2f} 分钟")
    print(f"  CRI (碰撞风险指数): {risk.cri:.3f}")
    print(f"  风险等级: {risk.risk_level.value.upper()}")
    print(f"  相对方位: {risk.relative_bearing:.1f}°")
    print(f"  当前距离: {risk.distance:.2f} 海里")
    print(f"  速度比: {risk.speed_ratio:.2f}")
    print()


def demo_low_risk_scenario():
    """演示低风险场景"""
    print_separator()
    print("场景 3: 低风险平行航行场景")
    print_separator()
    
    # 两船平行航行
    own_ship = ShipState(
        mmsi=123456789,
        latitude=0.0,
        longitude=0.0,
        heading=90.0,  # 向东
        sog=10.0
    )
    
    target_ship = ShipState(
        mmsi=987654321,
        latitude=0.1,  # 北侧约6海里
        longitude=0.0,
        heading=90.0,  # 向东
        sog=10.0
    )
    
    # 评估风险
    risk = assess_collision_risk(own_ship, target_ship)
    
    print(f"本船: MMSI={own_ship.mmsi}, 位置=({own_ship.latitude:.4f}, {own_ship.longitude:.4f})")
    print(f"      航向={own_ship.heading:.1f}°, 速度={own_ship.sog:.1f}节")
    print(f"目标船: MMSI={target_ship.mmsi}, 位置=({target_ship.latitude:.4f}, {target_ship.longitude:.4f})")
    print(f"        航向={target_ship.heading:.1f}°, 速度={target_ship.sog:.1f}节")
    print()
    print("风险评估结果:")
    print(f"  DCPA (最近会遇距离): {risk.dcpa:.2f} 海里")
    print(f"  TCPA (到达最近点时间): {risk.tcpa:.2f} 分钟 (inf表示永不相遇)")
    print(f"  CRI (碰撞风险指数): {risk.cri:.3f}")
    print(f"  风险等级: {risk.risk_level.value.upper()}")
    print(f"  相对方位: {risk.relative_bearing:.1f}°")
    print(f"  当前距离: {risk.distance:.2f} 海里")
    print(f"  速度比: {risk.speed_ratio:.2f}")
    print()


def demo_cri_factors():
    """演示CRI各因子的影响"""
    print_separator()
    print("场景 4: CRI因子影响分析")
    print_separator()
    
    # 基准场景
    dcpa = 1.5
    tcpa = 10.0
    bearing = 0.0
    speed_ratio = 1.0
    
    print("基准场景:")
    print(f"  DCPA = {dcpa} 海里")
    print(f"  TCPA = {tcpa} 分钟")
    print(f"  相对方位 = {bearing}°")
    print(f"  速度比 = {speed_ratio}")
    base_cri = calculate_cri(dcpa, tcpa, bearing, speed_ratio)
    print(f"  CRI = {base_cri:.3f}")
    print()
    
    # 测试距离因子影响
    print("距离因子影响 (DCPA变化):")
    for test_dcpa in [0.5, 1.0, 2.0, 5.0]:
        cri = calculate_cri(test_dcpa, tcpa, bearing, speed_ratio)
        print(f"  DCPA = {test_dcpa:.1f} 海里 -> CRI = {cri:.3f}")
    print()
    
    # 测试时间因子影响
    print("时间因子影响 (TCPA变化):")
    for test_tcpa in [3.0, 5.0, 10.0, 20.0]:
        cri = calculate_cri(dcpa, test_tcpa, bearing, speed_ratio)
        print(f"  TCPA = {test_tcpa:.1f} 分钟 -> CRI = {cri:.3f}")
    print()
    
    # 测试方位因子影响
    print("方位因子影响 (相对方位变化):")
    for test_bearing in [0.0, 45.0, 90.0, 135.0, 180.0]:
        cri = calculate_cri(dcpa, tcpa, test_bearing, speed_ratio)
        print(f"  方位 = {test_bearing:.1f}° -> CRI = {cri:.3f}")
    print()
    
    # 测试速度因子影响
    print("速度因子影响 (速度比变化):")
    for test_speed_ratio in [0.5, 1.0, 1.5, 2.0]:
        cri = calculate_cri(dcpa, tcpa, bearing, test_speed_ratio)
        print(f"  速度比 = {test_speed_ratio:.1f} -> CRI = {cri:.3f}")
    print()


def main():
    """主函数"""
    print()
    print("╔" + "═" * 78 + "╗")
    print("║" + " " * 20 + "CRI（碰撞风险指数）计算演示" + " " * 32 + "║")
    print("╚" + "═" * 78 + "╝")
    print()
    
    # 演示各种场景
    demo_high_risk_scenario()
    demo_medium_risk_scenario()
    demo_low_risk_scenario()
    demo_cri_factors()
    
    print_separator()
    print("演示完成！")
    print_separator()
    print()
    print("说明:")
    print("  - CRI值范围: 0.0 - 1.0")
    print("  - CRI < 0.5: 安全 (SAFE)")
    print("  - 0.5 ≤ CRI < 0.7: 预警 (WARNING)")
    print("  - CRI ≥ 0.7: 危险 (DANGER)")
    print()
    print("CRI计算公式:")
    print("  CRI = 0.4 × 距离因子 + 0.3 × 时间因子 + 0.2 × 方位因子 + 0.1 × 速度因子")
    print()


if __name__ == '__main__':
    main()
