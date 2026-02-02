#!/usr/bin/env python3
"""
演示 apply_colregs_rule() 函数的使用

展示如何根据不同的相遇类型应用COLREGS规则
"""

import sys
sys.path.insert(0, 'src/collision_avoidance')
sys.path.insert(0, 'src/scenario_generator')

from collision_avoidance.rules_engine import (
    apply_colregs_rule,
    determine_encounter_type,
    EncounterType,
    ActionType,
    TurnDirection
)
from scenario_generator.models import ShipState


def print_action(action, scenario_name):
    """打印避让动作信息"""
    print(f"\n{'='*60}")
    print(f"场景: {scenario_name}")
    print(f"{'='*60}")
    print(f"动作类型: {action.action_type.value}")
    if action.turn_direction:
        print(f"转向方向: {action.turn_direction.value}")
    if action.turn_angle:
        print(f"转向角度: {action.turn_angle}度")
    if action.speed_factor:
        print(f"速度因子: {action.speed_factor}")
    if action.maintain_course:
        print(f"保持航向: 是")
    if action.no_action:
        print(f"无需动作: 是")
    print(f"理由: {action.reason}")


def demo_head_on():
    """演示对遇场景"""
    own_ship = ShipState(
        mmsi=123456789,
        latitude=0.0,
        longitude=0.0,
        heading=0.0,  # 北向
        sog=10.0
    )
    target_ship = ShipState(
        mmsi=987654321,
        latitude=0.01,  # 北方
        longitude=0.0,
        heading=180.0,  # 南向
        sog=10.0
    )
    
    encounter_type = determine_encounter_type(own_ship, target_ship)
    action = apply_colregs_rule(encounter_type, own_ship, target_ship)
    print_action(action, "对遇场景 (Head-on)")


def demo_crossing_give_way():
    """演示交叉相遇场景 - 让路船"""
    own_ship = ShipState(
        mmsi=123456789,
        latitude=0.0,
        longitude=0.0,
        heading=0.0,  # 北向
        sog=10.0
    )
    target_ship = ShipState(
        mmsi=987654321,
        latitude=0.0,
        longitude=0.01,  # 右舷（东方）
        heading=270.0,  # 西向
        sog=10.0
    )
    
    encounter_type = determine_encounter_type(own_ship, target_ship)
    action = apply_colregs_rule(encounter_type, own_ship, target_ship)
    print_action(action, "交叉相遇场景 - 本船为让路船")


def demo_crossing_stand_on():
    """演示交叉相遇场景 - 直航船"""
    own_ship = ShipState(
        mmsi=123456789,
        latitude=0.0,
        longitude=0.0,
        heading=0.0,  # 北向
        sog=10.0
    )
    target_ship = ShipState(
        mmsi=987654321,
        latitude=0.0,
        longitude=-0.01,  # 左舷（西方）
        heading=90.0,  # 东向
        sog=10.0
    )
    
    encounter_type = determine_encounter_type(own_ship, target_ship)
    action = apply_colregs_rule(encounter_type, own_ship, target_ship)
    print_action(action, "交叉相遇场景 - 本船为直航船")


def demo_overtaking():
    """演示追越场景"""
    own_ship = ShipState(
        mmsi=123456789,
        latitude=0.0,
        longitude=0.0,
        heading=0.0,  # 北向
        sog=15.0  # 更快
    )
    target_ship = ShipState(
        mmsi=987654321,
        latitude=-0.01,  # 后方
        longitude=0.0,
        heading=0.0,  # 同向
        sog=10.0  # 更慢
    )
    
    encounter_type = determine_encounter_type(own_ship, target_ship)
    action = apply_colregs_rule(encounter_type, own_ship, target_ship)
    print_action(action, "追越场景 - 本船为追越船")


def demo_no_encounter():
    """演示无相遇风险场景"""
    own_ship = ShipState(
        mmsi=123456789,
        latitude=0.0,
        longitude=0.0,
        heading=0.0,  # 北向
        sog=10.0
    )
    target_ship = ShipState(
        mmsi=987654321,
        latitude=-0.01,  # 南方
        longitude=0.0,
        heading=0.0,  # 北向（背离）
        sog=10.0
    )
    
    encounter_type = determine_encounter_type(own_ship, target_ship)
    action = apply_colregs_rule(encounter_type, own_ship, target_ship)
    print_action(action, "无相遇风险场景")


def main():
    """主函数"""
    print("\n" + "="*60)
    print("COLREGS规则应用函数演示")
    print("="*60)
    
    # 演示各种场景
    demo_head_on()
    demo_crossing_give_way()
    demo_crossing_stand_on()
    demo_overtaking()
    demo_no_encounter()
    
    print("\n" + "="*60)
    print("演示完成！")
    print("="*60 + "\n")


if __name__ == "__main__":
    main()
