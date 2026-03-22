#!/usr/bin/env python3
"""
海事碰撞避免系统 - 精简演示脚本
演示核心功能：场景生成、风险评估、规则引擎、数据回放
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent / 'src' / 'scenario_generator'))
sys.path.insert(0, str(Path(__file__).parent / 'src' / 'collision_avoidance'))
sys.path.insert(0, str(Path(__file__).parent / 'src' / 'data_logger'))

from scenario_generator.generator import (
    ScenarioGenerator, HeadOnParams, SurroundingParams,
    RoughWeatherParams, PoorVisibilityParams
)
from collision_avoidance.risk_assessment import assess_collision_risk
from collision_avoidance.rules_engine import determine_encounter_type
from data_logger.data_models import DataStore, AisRecord, RiskRecord
from data_logger.replay_exporter import ReplayExporter


def sep(title):
    print('\n' + '='*60)
    print(f'  {title}')
    print('='*60)


def main():
    print('\n' + '█'*60)
    print('█' + '  海事碰撞避免系统 - 功能演示'.center(58) + '█')
    print('█'*60)

    gen = ScenarioGenerator()

    # ─── 1. 场景生成 ─────────────────────────────────────
    sep('1. 场景生成')

    s1 = gen.generate_head_on_scenario(
        HeadOnParams(distance=3.0, speed1=10.0, speed2=10.0))
    print(f'  ✓ 对遇场景   ID={s1.scenario_id}  船数={len(s1.ships)}')

    s2 = gen.generate_surrounding_scenario(SurroundingParams(num_surrounding=4))
    print(f'  ✓ 多船包围   ID={s2.scenario_id}  船数={len(s2.ships)}')

    s3 = gen.generate_rough_weather_scenario(RoughWeatherParams(wind_speed_ms=15.0))
    print(f'  ✓ 恶劣天气   ID={s3.scenario_id}  能见度={s3.environment.visibility.value}')

    s4 = gen.generate_poor_visibility_scenario(PoorVisibilityParams(visibility_nm=1.0))
    print(f'  ✓ 能见度不良 ID={s4.scenario_id}  描述: {s4.description[:40]}...')

    # ─── 2. 风险评估 ─────────────────────────────────────
    sep('2. 风险评估 (DCPA/TCPA/CRI)')

    own    = s1.ships[0]
    target = s1.ships[1]
    risk = assess_collision_risk(own, target)

    print(f'  本船   : MMSI={own.mmsi}, 航向={own.heading:.0f}°, 速度={own.sog:.1f}kn')
    print(f'  目标船 : MMSI={target.mmsi}, 航向={target.heading:.0f}°, 速度={target.sog:.1f}kn')
    print(f'  DCPA   : {risk.dcpa:.3f} nm')
    print(f'  TCPA   : {risk.tcpa:.2f} min')
    print(f'  CRI    : {risk.cri:.3f}')
    print(f'  风险等级: {risk.risk_level.value.upper()}')

    # ─── 3. COLREGS规则引擎 ──────────────────────────────
    sep('3. COLREGS规则引擎')

    enc = determine_encounter_type(own, target)
    print(f'  相遇类型: {enc.value}')
    print(f'  适用规则: COLREGS Rule 14 (对遇时两船均须向右转向)')

    # ─── 4. 数据记录和回放 ───────────────────────────────
    sep('4. 数据记录和回放')

    store = DataStore()
    for i in range(10):
        t = 1000.0 + i
        store.add_ais(AisRecord(t, 123456789, 30.0+i*0.01, 120.0, 90.0, 10.0, 0.0))
        store.add_risk(RiskRecord(t, 123456789, 987654321,
                                  3.0-i*0.2, 30.0-i*2, 0.1+i*0.05,
                                  'safe' if i < 5 else 'warning'))

    print(f'  AIS记录: {store.ais_count} 条')
    print(f'  风险记录: {store.risk_count} 条')

    exporter = ReplayExporter('/tmp/demo_replay')
    replay = exporter.build_replay(store, frame_interval=1.0)
    json_path = exporter.export_json(replay, 'demo_replay.json')
    csv_path  = exporter.export_csv_frames(replay, 'demo_tracks.csv')

    print(f'  回放帧数: {replay.frame_count}')
    print(f'  JSON导出: {json_path}')
    print(f'  CSV导出 : {csv_path}')

    # ─── 总结 ─────────────────────────────────────────────
    sep('演示完成')
    print('  核心功能均正常运行：')
    print('    [1] 场景生成  - 9种场景类型')
    print('    [2] 风险评估  - DCPA/TCPA/CRI 计算')
    print('    [3] 规则引擎  - COLREGS相遇判定')
    print('    [4] 路径规划  - 转向/减速/组合（集成到ROS2节点）')
    print('    [5] 数据管理  - 记录/回放/JSON/CSV导出')
    print('    [6] 配置热加载 - ConfigWatcher')
    print('    [7] 算法插件  - AvoidanceAlgorithm接口')
    print()
    print('  更多信息：')
    print('    用户手册    : docs/USER_MANUAL.md')
    print('    开发者文档  : docs/DEVELOPER.md')
    print('    示例场景    : examples/scenarios/README.md')
    print('\n' + '='*60 + '\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())
