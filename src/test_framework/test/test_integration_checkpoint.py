#!/usr/bin/env python3
"""Final Checkpoint - 系统集成测试"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent.parent / 'scenario_generator'))
sys.path.insert(0, str(Path(__file__).parent.parent.parent / 'collision_avoidance'))
sys.path.insert(0, str(Path(__file__).parent.parent.parent / 'data_logger'))

from scenario_generator.generator import ScenarioGenerator, HeadOnParams, SurroundingParams, RestrictedWaterParams, RoughWeatherParams, PoorVisibilityParams
from collision_avoidance.risk_assessment import assess_collision_risk
from data_logger.data_models import DataStore, AisRecord, RiskRecord
from data_logger.replay_exporter import ReplayExporter

class IntegrationTestRunner:
    def __init__(self):
        self.gen = ScenarioGenerator()
        self.passed = 0
        self.failed = 0

    def test_scenario(self, name, scenario_config):
        try:
            assert scenario_config is not None
            assert len(scenario_config.ships) >= 2
            assert scenario_config.duration > 0
            for ship in scenario_config.ships:
                assert 100000000 <= ship.mmsi <= 999999999
                assert -90 <= ship.latitude <= 90
                assert -180 <= ship.longitude <= 180
                assert 0 <= ship.heading < 360
                assert ship.sog >= 0
            own = scenario_config.ships[0]
            for target in scenario_config.ships[1:]:
                result = assess_collision_risk(own, target)
                assert result.dcpa >= 0 and 0 <= result.cri <= 1
            self.passed += 1
            print(f'  ✓ {name}')
            return True
        except Exception as e:
            self.failed += 1
            print(f'  ✗ {name}: {e}')
            return False

    def run_all_tests(self):
        print('\n' + '='*60)
        print('Final Checkpoint - 系统集成测试')
        print('='*60 + '\n')

        print('1. 基础场景生成测试')
        self.test_scenario('对遇', self.gen.generate_head_on_scenario(HeadOnParams(distance=3.0, speed1=10.0, speed2=10.0)))
        
        print('\n2. 扩展场景生成测试')
        self.test_scenario('包围', self.gen.generate_surrounding_scenario(SurroundingParams(num_surrounding=4)))
        self.test_scenario('受限水域', self.gen.generate_restricted_water_scenario(RestrictedWaterParams()))
        self.test_scenario('恶劣天气', self.gen.generate_rough_weather_scenario(RoughWeatherParams()))
        self.test_scenario('能见度不良', self.gen.generate_poor_visibility_scenario(PoorVisibilityParams()))

        print('\n3. 批量场景测试')
        for i in range(3):
            self.test_scenario(f'对遇-{i+1}', self.gen.generate_head_on_scenario(HeadOnParams(distance=2.0+i*0.5, speed1=10.0, speed2=10.0)))

        print('\n4. 数据记录和回放测试')
        try:
            store = DataStore()
            for i in range(10):
                store.add_ais(AisRecord(1000.0+i, 123456789, 30.0+i*0.01, 120.0, 90.0, 10.0, 0.0))
                store.add_risk(RiskRecord(1000.0+i, 123456789, 987654321, 2.0-i*0.1, 20.0-i, 0.2+i*0.05, 'safe'))
            exporter = ReplayExporter('/tmp/replay_test')
            replay = exporter.build_replay(store, frame_interval=1.0)
            assert replay.frame_count > 0
            print(f'  ✓ 数据回放 ({replay.frame_count}帧)')
            self.passed += 1
        except Exception as e:
            print(f'  ✗ 数据回放: {e}')
            self.failed += 1

        print('\n' + '='*60)
        print('测试结果总结')
        print('='*60)
        print(f'通过: {self.passed}')
        print(f'失败: {self.failed}')
        print('='*60)
        return self.failed == 0

if __name__ == '__main__':
    runner = IntegrationTestRunner()
    success = runner.run_all_tests()
    sys.exit(0 if success else 1)
