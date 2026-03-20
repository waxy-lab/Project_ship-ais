# 开发者文档

## 架构概览

系统由以下核心模块组成：

### 1. 场景生成器 (scenario_generator)
- **职责**：生成各种船舶相遇场景
- **核心类**：`ScenarioGenerator`
- **支持场景**：对遇、交叉、追越、多船、紧急、包围、受限水域、恶劣天气、能见度不良
- **输出**：`ScenarioConfig` 对象

### 2. 碰撞风险评估 (collision_avoidance)
- **职责**：计算DCPA、TCPA、CRI等风险指标
- **核心函数**：`assess_collision_risk()`
- **输出**：`RiskAssessment` 对象

### 3. 数据记录和回放 (data_logger)
- **职责**：记录AIS数据、风险数据、决策数据
- **核心类**：`DataStore`, `ReplayExporter`
- **输出**：JSON/CSV格式的回放文件

### 4. 可视化发布 (visualization_publisher_node)
- **职责**：实时发布关键参数到ROS2话题
- **发布话题**：
  - `/visualization/risk_parameters` - 风险参数
  - `/visualization/ship_summary` - 船舶摘要
  - `/visualization/alerts` - 告警信息

### 5. 配置热加载 (config_watcher)
- **职责**：监听配置文件变化，自动重新加载
- **核心类**：`ConfigWatcher`, `MultiConfigWatcher`
- **特性**：线程安全、支持回调、支持嵌套配置

### 6. 算法插件接口 (algorithm_plugin)
- **职责**：定义避碰算法的抽象接口
- **核心类**：`AvoidanceAlgorithm`, `AlgorithmRegistry`
- **特性**：动态加载、单例管理、模块扫描

## 扩展指南

### 添加新的场景类型

1. 在 `scenario_generator/generator.py` 中定义参数类：

```python
@dataclass
class MyScenarioParams:
    """我的场景参数"""
    param1: float = 1.0
    param2: int = 10
    
    def __post_init__(self):
        # 参数验证
        if self.param1 <= 0:
            raise ValueError("param1 必须大于0")
```

2. 在 `ScenarioGenerator` 类中实现生成方法：

```python
def generate_my_scenario(self, params: MyScenarioParams) -> ScenarioConfig:
    """生成我的场景"""
    # 创建船舶
    own_ship = ShipState(...)
    target_ship = ShipState(...)
    
    # 创建环境
    env = EnvironmentConfig(...)
    
    # 返回场景
    return ScenarioConfig(
        scenario_id=...,
        scenario_type=ScenarioType.MY_TYPE,
        ships=[own_ship, target_ship],
        environment=env,
        duration=params.duration,
        description=...
    )
```

3. 编写单元测试：

```python
def test_my_scenario(gen):
    params = MyScenarioParams(param1=2.0)
    scenario = gen.generate_my_scenario(params)
    assert len(scenario.ships) == 2
    assert scenario.duration > 0
```

### 实现自定义避碰算法

1. 继承 `AvoidanceAlgorithm`：

```python
from collision_avoidance.algorithm_plugin import AvoidanceAlgorithm, AvoidanceStrategy

class MyAlgorithm(AvoidanceAlgorithm):
    def __init__(self):
        super().__init__('my_algorithm', '1.0')
    
    def compute_strategy(self, own_ship, target_ships, environment):
        # 计算避碰策略
        if not target_ships:
            return None
        
        target = target_ships[0]
        # ... 计算逻辑 ...
        
        return AvoidanceStrategy(
            strategy_id='my_001',
            algorithm_name=self.name,
            target_heading=100.0,
            target_speed=8.0,
            reason='my algorithm decision',
            confidence=0.8
        )
    
    def validate_inputs(self, own_ship, target_ships):
        return own_ship is not None and len(target_ships) > 0
```

2. 注册算法：

```python
from collision_avoidance.algorithm_plugin import get_global_registry

registry = get_global_registry()
registry.register('my_algo', MyAlgorithm)

# 使用
algo = registry.get_algorithm('my_algo')
strategy = algo.compute_strategy(own, targets, env)
```

## 贡献指南

### 代码风格

- 遵循 PEP 8 规范
- 使用类型注解
- 添加文档字符串
- 编写单元测试

### 提交流程

1. 创建特性分支：`git checkout -b feature/my-feature`
2. 提交更改：`git commit -m "feat: 描述"`
3. 推送分支：`git push origin feature/my-feature`
4. 创建 Pull Request

### 测试要求

- 所有新代码必须有单元测试
- 测试覆盖率 >= 80%
- 所有测试必须通过
- 无 linter 错误

### 文档要求

- 更新相关文档
- 添加使用示例
- 更新 CHANGELOG

## 性能优化

### 场景生成

- 使用缓存避免重复计算
- 批量生成时使用多进程

### 风险评估

- 预计算常用参数
- 使用向量化操作

### 数据回放

- 按需加载数据
- 使用流式处理大文件

## 常见问题

**Q: 如何添加新的环境参数？**
A: 修改 `EnvironmentConfig` 数据类，添加新字段并更新验证逻辑。

**Q: 如何支持新的船舶类型？**
A: 扩展 `ShipState` 类或创建子类，添加特定属性。

**Q: 如何集成外部数据源？**
A: 实现数据适配器，将外部数据转换为 `ShipState` 对象。

## 参考资源

- 设计文档：`docs/design.md`
- API文档：`docs/api.md`
- 用户手册：`docs/USER_MANUAL.md`
