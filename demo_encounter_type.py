#!/usr/bin/env python3
"""
相遇类型判定演示脚本

演示 determine_encounter_type() 函数的使用
"""

import sys
import os

# 添加源代码路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src/scenario_generator'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src/collision_avoidance'))

from scenario_generator.models import ShipState
from collision_avoidance.rules_engine import (
    determine_encounter_type,
    determine_vessel_roles,
    analyze_encounter_situation,
    EncounterType,
    VesselRole,
)


def print_separator():
    print("\n" + "="*80 + "\n")


def demo_head_on():
    """演示对遇场景"""
    print("场景 1: 对遇 (Head-On)")
    print("-" * 40)
    
    own_ship = ShipState(
        mmsi=123456789,
        latitude=30.0,
        longitude=120.0,
        heading=0.0,  # 北向
        sog=10.0
    )
    
    target_ship = ShipState(
        mmsi=987654321,
        latitude=30.1,  # 北方约6海里
        longitude=120.0,
        heading=180.0,  # 南向（相向）
        sog=12.0
    )
    
    print(f"本船: MMSI={own_ship.mmsi}, 位置=({own_ship.latitude:.2f}, {own_ship.longitude:.2f}), "
          f"航向={own_ship.heading:.0f}°, 速度={own_ship.speed:.1f}节")
    print(f"目标船: MMSI={target_ship.mmsi}, 位置=({target_ship.latitude:.2f}, {target_ship.longitude:.2f}), "
          f"航向={target_ship.heading:.0f}°, 速度={target_ship.speed:.1f}节")
    
    # 判定相遇类型
    encounter_type = determine_encounter_type(own_ship, target_ship)
    print(f"\n相遇类型: {encounter_type.value}")
    
    # 确定船舶角色
    own_role, target_role = determine_vessel_roles(encounter_type, own_ship, target_ship)
    print(f"本船角色: {own_role.value}")
    print(f"目标船角色: {target_role.value}")
    
    # 分析态势
    situation = analyze_encounter_situation(own_ship, target_ship)
    print(f"\n态势分析:")
    print(f"  相对方位: {situation.relative_bearing:.1f}°")
    print(f"  航向差: {situation.heading_difference:.1f}°")
    print(f"  距离: {situation.distance:.2f}海里")
    print(f"  是否追越: {situation.is_overtaking}")
    print(f"  是否从右舷交叉: {situation.is_crossing_from_starboard}")


def demo_crossing():
    """演示交叉场景"""
    print("场景 2: 交叉相遇 (Crossing)")
    print("-" * 40)
    
    own_ship = ShipState(
        mmsi=123456789,
        latitude=30.0,
        longitude=120.0,
        heading=0.0,  # 北向
        sog=10.0
    )
    
    target_ship = ShipState(
        mmsi=987654321,
        latitude=30.0,
        longitude=120.1,  # 东方约6海里
        heading=270.0,  # 西向（交叉）
        sog=12.0
    )
    
    print(f"本船: MMSI={own_ship.mmsi}, 位置=({own_ship.latitude:.2f}, {own_ship.longitude:.2f}), "
          f"航向={own_ship.heading:.0f}°, 速度={own_ship.speed:.1f}节")
    print(f"目标船: MMSI={target_ship.mmsi}, 位置=({target_ship.latitude:.2f}, {target_ship.longitude:.2f}), "
          f"航向={target_ship.heading:.0f}°, 速度={target_ship.speed:.1f}节")
    
    # 判定相遇类型
    encounter_type = determine_encounter_type(own_ship, target_ship)
    print(f"\n相遇类型: {encounter_type.value}")
    
    # 确定船舶角色
    own_role, target_role = determine_vessel_roles(encounter_type, own_ship, target_ship)
    print(f"本船角色: {own_role.value}")
    print(f"目标船角色: {target_role.value}")
    
    # 分析态势
    situation = analyze_encounter_situation(own_ship, target_ship)
    print(f"\n态势分析:")
    print(f"  相对方位: {situation.relative_bearing:.1f}°")
    print(f"  航向差: {situation.heading_difference:.1f}°")
    print(f"  距离: {situation.distance:.2f}海里")
    print(f"  是否追越: {situation.is_overtaking}")
    print(f"  是否从右舷交叉: {situation.is_crossing_from_starboard}")


def demo_overtaking():
    """演示追越场景"""
    print("场景 3: 追越 (Overtaking)")
    print("-" * 40)
    
    own_ship = ShipState(
        mmsi=123456789,
        latitude=30.0,
        longitude=120.0,
        heading=0.0,  # 北向
        sog=15.0  # 更快
    )
    
    target_ship = ShipState(
        mmsi=987654321,
        latitude=29.95,  # 后方约3海里
        longitude=120.0,
        heading=0.0,  # 同向
        sog=10.0  # 更慢
    )
    
    print(f"本船: MMSI={own_ship.mmsi}, 位置=({own_ship.latitude:.2f}, {own_ship.longitude:.2f}), "
          f"航向={own_ship.heading:.0f}°, 速度={own_ship.speed:.1f}节")
    print(f"目标船: MMSI={target_ship.mmsi}, 位置=({target_ship.latitude:.2f}, {target_ship.longitude:.2f}), "
          f"航向={target_ship.heading:.0f}°, 速度={target_ship.speed:.1f}节")
    
    # 判定相遇类型
    encounter_type = determine_encounter_type(own_ship, target_ship)
    print(f"\n相遇类型: {encounter_type.value}")
    
    # 确定船舶角色
    own_role, target_role = determine_vessel_roles(encounter_type, own_ship, target_ship)
    print(f"本船角色: {own_role.value}")
    print(f"目标船角色: {target_role.value}")
    
    # 分析态势
    situation = analyze_encounter_situation(own_ship, target_ship)
    print(f"\n态势分析:")
    print(f"  相对方位: {situation.relative_bearing:.1f}°")
    print(f"  航向差: {situation.heading_difference:.1f}°")
    print(f"  距离: {situation.distance:.2f}海里")
    print(f"  是否追越: {situation.is_overtaking}")
    print(f"  是否从右舷交叉: {situation.is_crossing_from_starboard}")


def demo_no_encounter():
    """演示无相遇风险场景"""
    print("场景 4: 无相遇风险 (No Encounter)")
    print("-" * 40)
    
    own_ship = ShipState(
        mmsi=123456789,
        latitude=30.0,
        longitude=120.0,
        heading=0.0,  # 北向
        sog=10.0
    )
    
    target_ship = ShipState(
        mmsi=987654321,
        latitude=30.0,
        longitude=120.5,  # 远处东方约30海里
        heading=0.0,  # 同向平行
        sog=10.0
    )
    
    print(f"本船: MMSI={own_ship.mmsi}, 位置=({own_ship.latitude:.2f}, {own_ship.longitude:.2f}), "
          f"航向={own_ship.heading:.0f}°, 速度={own_ship.speed:.1f}节")
    print(f"目标船: MMSI={target_ship.mmsi}, 位置=({target_ship.latitude:.2f}, {target_ship.longitude:.2f}), "
          f"航向={target_ship.heading:.0f}°, 速度={target_ship.speed:.1f}节")
    
    # 判定相遇类型
    encounter_type = determine_encounter_type(own_ship, target_ship)
    print(f"\n相遇类型: {encounter_type.value}")
    
    # 确定船舶角色
    own_role, target_role = determine_vessel_roles(encounter_type, own_ship, target_ship)
    print(f"本船角色: {own_role.value}")
    print(f"目标船角色: {target_role.value}")
    
    # 分析态势
    situation = analyze_encounter_situation(own_ship, target_ship)
    print(f"\n态势分析:")
    print(f"  相对方位: {situation.relative_bearing:.1f}°")
    print(f"  航向差: {situation.heading_difference:.1f}°")
    print(f"  距离: {situation.distance:.2f}海里")
    print(f"  是否追越: {situation.is_overtaking}")
    print(f"  是否从右舷交叉: {situation.is_crossing_from_starboard}")


def main():
    """主函数"""
    print("\n" + "="*80)
    print("相遇类型判定演示")
    print("="*80)
    
    print_separator()
    demo_head_on()
    
    print_separator()
    demo_crossing()
    
    print_separator()
    demo_overtaking()
    
    print_separator()
    demo_no_encounter()
    
    print_separator()
    print("演示完成！")
    print("="*80 + "\n")


if __name__ == "__main__":
    main()
