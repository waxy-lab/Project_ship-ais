#!/usr/bin/env python3
"""
批量测试运行脚本
运行多个避碰场景并生成性能对比报告
Requirements: 6.6
"""
import sys
import os
import argparse

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../scenario_generator'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../collision_avoidance'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../test_framework'))

from scenario_generator.models import (
    ShipState, ScenarioConfig, ScenarioType, EnvironmentConfig
)
from test_framework.test_runner import TestRunner
from test_framework.metrics_calculator import compute_batch_report
from test_framework.report_generator import ReportGenerator


def make_ship(mmsi, lat, lon, hdg, sog):
    return ShipState(mmsi=mmsi, latitude=lat, longitude=lon,
                     heading=hdg, sog=sog, rot=0.0)


def build_scenarios():
    """构建标准测试场景集"""
    scenarios = []
    # 场景1: 基础对遇
    scenarios.append(ScenarioConfig(
        scenario_id="head_on_basic",
        scenario_type=ScenarioType.HEAD_ON,
        ships=[make_ship(123456789, 30.0, 120.0, 90.0, 15.0),
               make_ship(987654321, 30.0, 120.15, 270.0, 15.0)],
        duration=300.0, description="基础对遇场景",
    ))
    # 场景2: 高速对遇
    scenarios.append(ScenarioConfig(
        scenario_id="head_on_high_speed",
        scenario_type=ScenarioType.HEAD_ON,
        ships=[make_ship(111111111, 30.0, 120.0, 90.0, 25.0),
               make_ship(222222222, 30.0, 120.2, 270.0, 25.0)],
        duration=300.0, description="高速对遇场景",
    ))
    # 场景3: 交叉相遇
    scenarios.append(ScenarioConfig(
        scenario_id="crossing_basic",
        scenario_type=ScenarioType.CROSSING,
        ships=[make_ship(333333333, 30.0, 120.0, 0.0, 12.0),
               make_ship(444444444, 30.083, 120.05, 270.0, 12.0)],
        duration=300.0, description="交叉相遇场景",
    ))
    # 场景4: 追越
    scenarios.append(ScenarioConfig(
        scenario_id="overtaking_basic",
        scenario_type=ScenarioType.OVERTAKING,
        ships=[make_ship(555555555, 30.0, 120.0, 90.0, 20.0),
               make_ship(666666666, 30.0, 120.05, 90.0, 8.0)],
        duration=300.0, description="追越场景",
    ))
    # 场景5: 近距离对遇
    scenarios.append(ScenarioConfig(
        scenario_id="head_on_close",
        scenario_type=ScenarioType.HEAD_ON,
        ships=[make_ship(777777777, 30.0, 120.0, 90.0, 12.0),
               make_ship(888888888, 30.0, 120.05, 270.0, 12.0)],
        duration=200.0, description="近距离紧急对遇",
    ))
    return scenarios


def main():
    parser = argparse.ArgumentParser(description='批量避碰测试')
    parser.add_argument('--output-dir', default='/tmp/collision_avoidance_reports',
                        help='报告输出目录')
    parser.add_argument('--step', type=float, default=10.0,
                        help='仿真步长（秒）')
    args = parser.parse_args()

    print("=" * 60)
    print("  避碰算法批量性能测试")
    print("=" * 60)

    scenarios = build_scenarios()
    print(f"\n共 {len(scenarios)} 个测试场景")

    runner = TestRunner(own_ship_index=0, step_sec=args.step)
    print("\n正在运行场景...")
    metrics_list = []
    for sc in scenarios:
        print(f"  [{sc.scenario_id}] {sc.description}...", end=' ', flush=True)
        try:
            m = runner.run_scenario(sc)
            metrics_list.append(m)
            status = "SUCCESS" if m.is_successful else "COLLISION"
            print(f"{status} | DCPA={m.min_dcpa_nm:.3f}nm | "
                  f"改向{m.course_change_count}次")
        except Exception as e:
            print(f"ERROR: {e}")

    report = compute_batch_report(metrics_list, "batch_001")
    gen = ReportGenerator(output_dir=args.output_dir)
    files = gen.generate_all(report, prefix="collision_avoidance_report")

    print("\n" + "=" * 60)
    print(f"  成功率: {report.success_rate*100:.1f}%  "
          f"({report.success_count}/{report.total_scenarios})")
    print(f"  平均最小DCPA: {report.avg_min_dcpa_nm:.3f} nm")
    print(f"  平均航向改变: {report.avg_course_change_count:.1f} 次")
    print(f"  COLREGS遵守率: {report.avg_colregs_compliance_rate*100:.1f}%")
    print("=" * 60)
    print("\n报告文件:")
    for fmt, path in files.items():
        print(f"  {fmt.upper():4s}: {path}")


if __name__ == '__main__':
    main()
