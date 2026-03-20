"""
避碰算法插件接口

定义避碰算法的抽象接口，支持动态加载自定义算法。
Requirements: 8.3
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Type
import importlib
import inspect
import logging

from scenario_generator.models import ShipState, EnvironmentConfig

logger = logging.getLogger(__name__)


@dataclass
class AvoidanceStrategy:
    """避碰策略结果"""
    strategy_id: str
    algorithm_name: str
    target_heading: float
    target_speed: float
    reason: str
    confidence: float  # 0-1，置信度
    metadata: Dict[str, Any] = None


class AvoidanceAlgorithm(ABC):
    """
    避碰算法抽象基类

    所有避碰算法必须继承此类并实现 compute_strategy 方法。
    Requirements: 8.3
    """

    def __init__(self, name: str, version: str = "1.0"):
        """
        初始化算法

        Args:
            name: 算法名称
            version: 算法版本
        """
        self.name = name
        self.version = version
        self._config: Dict[str, Any] = {}

    @property
    def algorithm_name(self) -> str:
        return self.name

    @property
    def algorithm_version(self) -> str:
        return self.version

    @property
    def config(self) -> Dict[str, Any]:
        return dict(self._config)

    def set_config(self, config: Dict[str, Any]):
        """设置算法配置参数"""
        self._config = dict(config)

    @abstractmethod
    def compute_strategy(
            self,
            own_ship: ShipState,
            target_ships: List[ShipState],
            environment: EnvironmentConfig,
    ) -> Optional[AvoidanceStrategy]:
        """
        计算避碰策略

        Args:
            own_ship: 本船状态
            target_ships: 目标船列表
            environment: 环境配置

        Returns:
            AvoidanceStrategy 或 None（无需避让）
        """
        pass

    @abstractmethod
    def validate_inputs(
            self,
            own_ship: ShipState,
            target_ships: List[ShipState],
    ) -> bool:
        """
        验证输入数据有效性

        Returns:
            True 表示有效，False 表示无效
        """
        pass

    def __repr__(self) -> str:
        return f'{self.__class__.__name__}(name={self.name!r}, version={self.version!r})'


class AlgorithmRegistry:
    """
    算法注册表

    管理所有可用的避碰算法，支持动态注册和加载。
    Requirements: 8.3
    """

    def __init__(self):
        self._algorithms: Dict[str, Type[AvoidanceAlgorithm]] = {}
        self._instances: Dict[str, AvoidanceAlgorithm] = {}

    def register(
            self,
            name: str,
            algorithm_class: Type[AvoidanceAlgorithm],
    ):
        """
        注册算法类

        Args:
            name: 算法注册名
            algorithm_class: 算法类（必须继承 AvoidanceAlgorithm）
        """
        if not issubclass(algorithm_class, AvoidanceAlgorithm):
            raise TypeError(
                f'{algorithm_class} 必须继承 AvoidanceAlgorithm')
        self._algorithms[name] = algorithm_class
        logger.info(f'算法已注册: {name} -> {algorithm_class.__name__}')

    def unregister(self, name: str):
        """注销算法"""
        if name in self._algorithms:
            del self._algorithms[name]
            if name in self._instances:
                del self._instances[name]
            logger.info(f'算法已注销: {name}')

    def get_algorithm(self, name: str) -> Optional[AvoidanceAlgorithm]:
        """
        获取算法实例（单例）

        Args:
            name: 算法注册名

        Returns:
            AvoidanceAlgorithm 实例或 None
        """
        if name not in self._algorithms:
            logger.warning(f'算法未注册: {name}')
            return None
        if name not in self._instances:
            try:
                self._instances[name] = self._algorithms[name]()
            except Exception as e:
                logger.error(f'算法实例化失败 {name}: {e}')
                return None
        return self._instances[name]

    def list_algorithms(self) -> List[str]:
        """列出所有已注册算法"""
        return list(self._algorithms.keys())

    def load_from_module(self, module_path: str) -> int:
        """
        从模块动态加载算法

        扫描模块中所有 AvoidanceAlgorithm 子类并自动注册。

        Args:
            module_path: 模块路径（如 'my_package.my_algorithms'）

        Returns:
            成功加载的算法数量
        """
        try:
            module = importlib.import_module(module_path)
        except ImportError as e:
            logger.error(f'模块加载失败 {module_path}: {e}')
            return 0

        count = 0
        for name, obj in inspect.getmembers(module):
            if (inspect.isclass(obj) and
                    issubclass(obj, AvoidanceAlgorithm) and
                    obj is not AvoidanceAlgorithm):
                try:
                    self.register(name, obj)
                    count += 1
                except Exception as e:
                    logger.warning(f'注册失败 {name}: {e}')
        return count

    def __repr__(self) -> str:
        return f'AlgorithmRegistry(algorithms={list(self._algorithms.keys())})'


# 全局注册表实例
_global_registry = AlgorithmRegistry()


def get_global_registry() -> AlgorithmRegistry:
    """获取全局算法注册表"""
    return _global_registry


def register_algorithm(
        name: str,
        algorithm_class: Type[AvoidanceAlgorithm],
):
    """
    向全局注册表注册算法

    使用示例：
        @register_algorithm('my_algo')
        class MyAlgorithm(AvoidanceAlgorithm):
            ...
    """
    _global_registry.register(name, algorithm_class)
    return algorithm_class
