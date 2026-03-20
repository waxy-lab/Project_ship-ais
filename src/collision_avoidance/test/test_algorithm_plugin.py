"""
算法插件接口单元测试
Requirements: 8.3
"""
import pytest
from collision_avoidance.algorithm_plugin import (
    AvoidanceAlgorithm, AvoidanceStrategy, AlgorithmRegistry,
    register_algorithm, get_global_registry
)
from scenario_generator.models import ShipState, EnvironmentConfig


class DummyAlgorithm(AvoidanceAlgorithm):
    """测试用算法"""
    def compute_strategy(self, own_ship, target_ships, environment):
        if not target_ships:
            return None
        return AvoidanceStrategy(
            strategy_id='dummy_001',
            algorithm_name=self.name,
            target_heading=90.0,
            target_speed=8.0,
            reason='dummy strategy',
            confidence=0.5,
        )

    def validate_inputs(self, own_ship, target_ships):
        return own_ship is not None


class TestAvoidanceStrategy:
    def test_creation(self):
        s = AvoidanceStrategy(
            strategy_id='s1',
            algorithm_name='test',
            target_heading=45.0,
            target_speed=10.0,
            reason='test',
            confidence=0.8,
        )
        assert s.strategy_id == 's1'
        assert s.confidence == 0.8

    def test_metadata_default_none(self):
        s = AvoidanceStrategy(
            strategy_id='s1',
            algorithm_name='test',
            target_heading=0.0,
            target_speed=0.0,
            reason='',
            confidence=0.0,
        )
        assert s.metadata is None


class TestAvoidanceAlgorithm:
    def test_cannot_instantiate_abstract(self):
        with pytest.raises(TypeError):
            AvoidanceAlgorithm('test')

    def test_dummy_algorithm_works(self):
        algo = DummyAlgorithm('dummy', '1.0')
        assert algo.name == 'dummy'
        assert algo.version == '1.0'

    def test_algorithm_name_property(self):
        algo = DummyAlgorithm('test_algo')
        assert algo.algorithm_name == 'test_algo'

    def test_algorithm_version_property(self):
        algo = DummyAlgorithm('test', '2.5')
        assert algo.algorithm_version == '2.5'

    def test_set_config(self):
        algo = DummyAlgorithm('test')
        algo.set_config({'param1': 10, 'param2': 'value'})
        assert algo.config['param1'] == 10

    def test_config_returns_copy(self):
        algo = DummyAlgorithm('test')
        algo.set_config({'key': 'value'})
        cfg1 = algo.config
        cfg2 = algo.config
        assert cfg1 is not cfg2

    def test_compute_strategy_returns_strategy(self):
        algo = DummyAlgorithm('test')
        own = ShipState(123456789, 30.0, 120.0, 0.0, 10.0)
        target = ShipState(987654321, 30.1, 120.0, 180.0, 8.0)
        env = EnvironmentConfig()
        result = algo.compute_strategy(own, [target], env)
        assert isinstance(result, AvoidanceStrategy)

    def test_compute_strategy_no_targets(self):
        algo = DummyAlgorithm('test')
        own = ShipState(123456789, 30.0, 120.0, 0.0, 10.0)
        env = EnvironmentConfig()
        result = algo.compute_strategy(own, [], env)
        assert result is None

    def test_validate_inputs(self):
        algo = DummyAlgorithm('test')
        own = ShipState(123456789, 30.0, 120.0, 0.0, 10.0)
        assert algo.validate_inputs(own, []) is True

    def test_repr(self):
        algo = DummyAlgorithm('test', '1.0')
        r = repr(algo)
        assert 'DummyAlgorithm' in r
        assert 'test' in r


class TestAlgorithmRegistry:
    def test_register_algorithm(self):
        reg = AlgorithmRegistry()
        reg.register('dummy', DummyAlgorithm)
        assert 'dummy' in reg.list_algorithms()

    def test_register_non_algorithm_raises(self):
        reg = AlgorithmRegistry()
        class NotAlgorithm:
            pass
        with pytest.raises(TypeError):
            reg.register('bad', NotAlgorithm)

    def test_get_algorithm_singleton(self):
        reg = AlgorithmRegistry()
        reg.register('dummy', DummyAlgorithm)
        a1 = reg.get_algorithm('dummy')
        a2 = reg.get_algorithm('dummy')
        assert a1 is a2

    def test_get_nonexistent_returns_none(self):
        reg = AlgorithmRegistry()
        assert reg.get_algorithm('nonexistent') is None

    def test_unregister(self):
        reg = AlgorithmRegistry()
        reg.register('dummy', DummyAlgorithm)
        assert 'dummy' in reg.list_algorithms()
        reg.unregister('dummy')
        assert 'dummy' not in reg.list_algorithms()

    def test_list_algorithms(self):
        reg = AlgorithmRegistry()
        reg.register('algo1', DummyAlgorithm)
        reg.register('algo2', DummyAlgorithm)
        algos = reg.list_algorithms()
        assert 'algo1' in algos
        assert 'algo2' in algos

    def test_repr(self):
        reg = AlgorithmRegistry()
        reg.register('test', DummyAlgorithm)
        r = repr(reg)
        assert 'AlgorithmRegistry' in r

    def test_load_from_module_invalid(self):
        reg = AlgorithmRegistry()
        count = reg.load_from_module('nonexistent.module')
        assert count == 0

    def test_load_from_module_builtin(self):
        reg = AlgorithmRegistry()
        # 加载 json 模块（不包含 AvoidanceAlgorithm）
        count = reg.load_from_module('json')
        assert count == 0


class TestGlobalRegistry:
    def test_get_global_registry(self):
        reg = get_global_registry()
        assert isinstance(reg, AlgorithmRegistry)

    def test_register_algorithm_direct(self):
        class TestAlgo(AvoidanceAlgorithm):
            def compute_strategy(self, own, targets, env):
                return None
            def validate_inputs(self, own, targets):
                return True

        reg = get_global_registry()
        reg.register('test_algo_direct', TestAlgo)
        assert 'test_algo_direct' in reg.list_algorithms()


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
