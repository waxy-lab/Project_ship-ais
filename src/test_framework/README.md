# 测试框架 (Test Framework)

## 概述

本测试框架为水路仿真测试系统提供了完整的测试工具和策略生成器，支持：

- **单元测试**：验证具体功能和边界情况
- **基于属性的测试（Property-Based Testing）**：使用 Hypothesis 验证通用属性
- **集成测试**：验证模块间的协同工作

## 安装依赖

```bash
# 安装测试依赖
pip install pytest hypothesis pytest-timeout pytest-cov

# 或使用项目的 requirements.txt
pip install -r requirements.txt
```

## 目录结构

```
test_framework/
├── test_framework/
│   ├── __init__.py           # 模块导出
│   ├── strategies.py         # Hypothesis 策略生成器
│   └── test_helpers.py       # 测试工具函数
├── test/
│   ├── __init__.py
│   └── test_example.py       # 示例测试
├── README.md                 # 本文档
└── setup.py                  # 包配置
```

## 使用方法

### 1. 导入测试工具

```python
from test_framework.test_framework import (
    # Hypothesis 策略
    ship_state_strategy,
    head_on_params_strategy,
    latitude_strategy,
    longitude_strategy,
    
    # 工具函数
    calculate_distance,
    calculate_bearing,
    calculate_dcpa_tcpa,
    is_head_on_situation,
    assert_valid_ship_state,
)
```

### 2. 编写单元测试

```python
import pytest

@pytest.mark.unit
def test_calculate_distance():
    """测试距离计算"""
    distance = calculate_distance(30.0, 120.0, 31.0, 120.0)
    assert distance == pytest.approx(60.0, rel=0.01)
```

### 3. 编写基于属性的测试

```python
from hypothesis import given, settings

@pytest.mark.property
@given(ship=ship_state_strategy())
@settings(max_examples=100)
def test_property_ship_state_valid(ship):
    """属性：生成的船舶状态应该有效"""
    assert_valid_ship_state(ship)
    assert -90 <= ship['latitude'] <= 90
    assert ship['speed'] >= 0
```

### 4. 运行测试

```bash
# 运行所有测试
PYTHONPATH=src:$PYTHONPATH pytest

# 运行特定测试文件
PYTHONPATH=src:$PYTHONPATH pytest src/test_framework/test/test_example.py

# 运行特定测试类
PYTHONPATH=src:$PYTHONPATH pytest src/test_framework/test/test_example.py::TestGeometryCalculations

# 运行带标记的测试
PYTHONPATH=src:$PYTHONPATH pytest -m unit          # 只运行单元测试
PYTHONPATH=src:$PYTHONPATH pytest -m property      # 只运行属性测试
PYTHONPATH=src:$PYTHONPATH pytest -m integration   # 只运行集成测试

# 详细输出
PYTHONPATH=src:$PYTHONPATH pytest -v

# 显示测试覆盖率
PYTHONPATH=src:$PYTHONPATH pytest --cov=test_framework --cov-report=html
```

## Hypothesis 策略说明

### 基础策略

- `latitude_strategy()` - 生成有效纬度 (-90 到 90 度)
- `longitude_strategy()` - 生成有效经度 (-180 到 180 度)
- `heading_strategy()` - 生成有效航向 (0 到 360 度)
- `speed_strategy()` - 生成有效速度 (0 到 30 节)
- `mmsi_strategy()` - 生成有效 MMSI 号码

### 船舶状态策略

```python
@given(ship=ship_state_strategy())
def test_example(ship):
    # ship 包含: mmsi, latitude, longitude, heading, speed, etc.
    pass

# 自定义范围
@given(ship=ship_state_strategy(
    lat_range=(20.0, 50.0),
    lon_range=(100.0, 140.0),
    speed_range=(5.0, 20.0)
))
def test_example_with_range(ship):
    pass
```

### 场景参数策略

- `head_on_params_strategy()` - 对遇场景参数
- `crossing_params_strategy()` - 交叉相遇场景参数
- `overtaking_params_strategy()` - 追越场景参数
- `emergency_params_strategy()` - 紧急避让场景参数

### 完整场景策略

```python
@given(scenario=head_on_scenario_strategy())
def test_scenario(scenario):
    # scenario 包含: scenario_type, ships, environment, duration
    assert len(scenario['ships']) == 2
```

## 测试工具函数

### 几何计算

- `calculate_distance(lat1, lon1, lat2, lon2)` - 计算两点距离（海里）
- `calculate_bearing(lat1, lon1, lat2, lon2)` - 计算方位角（度）
- `calculate_relative_bearing(own_heading, target_bearing)` - 计算相对方位
- `normalize_angle(angle)` - 角度归一化到 0-360 度
- `angle_difference(angle1, angle2)` - 计算角度差

### 船舶运动计算

- `calculate_velocity_components(speed, heading)` - 计算速度分量
- `calculate_dcpa_tcpa(ship1, ship2)` - 计算 DCPA 和 TCPA

### 场景验证

- `is_head_on_situation(ship1, ship2)` - 判断是否为对遇局面
- `is_crossing_situation(ship1, ship2)` - 判断是否为交叉相遇
- `is_overtaking_situation(ship1, ship2)` - 判断是否为追越局面

### 断言工具

- `assert_valid_ship_state(ship)` - 断言船舶状态有效
- `assert_valid_scenario(scenario)` - 断言场景配置有效
- `assert_angle_in_range(angle, min, max)` - 断言角度在范围内
- `assert_approximately_equal(v1, v2, tolerance)` - 断言近似相等

### 性能指标

- `calculate_collision_risk_index(dcpa, tcpa)` - 计算碰撞风险指数
- `calculate_path_efficiency(original, actual)` - 计算路径效率

## pytest 配置

项目根目录的 `pytest.ini` 文件包含了测试配置：

```ini
[pytest]
# 测试文件搜索模式
python_files = test_*.py *_test.py
python_classes = Test* *Tests
python_functions = test_*

# 测试目录
testpaths = 
    src/scenario_generator/test
    src/collision_avoidance/test
    src/test_framework/test
    src/ais_simulator/test

# 标记定义
markers =
    unit: 单元测试
    property: 基于属性的测试（Hypothesis）
    integration: 集成测试
    slow: 运行时间较长的测试
    pbt: Property-Based Test 标记

[hypothesis]
max_examples = 100
deadline = None
print_blob = True
```

## 最佳实践

### 1. 测试命名

- 单元测试：`test_<function_name>_<scenario>`
- 属性测试：`test_property_<property_description>`
- 集成测试：`test_integration_<workflow>`

### 2. 使用标记

```python
@pytest.mark.unit
def test_unit_example():
    pass

@pytest.mark.property
@given(...)
def test_property_example():
    pass

@pytest.mark.integration
def test_integration_example():
    pass

@pytest.mark.slow
def test_slow_example():
    pass
```

### 3. 属性测试配置

```python
from hypothesis import given, settings

@given(...)
@settings(
    max_examples=100,      # 测试用例数量
    deadline=None,         # 禁用超时
)
def test_property():
    pass
```

### 4. 测试组织

- 将相关测试放在同一个测试类中
- 使用描述性的测试类名和方法名
- 添加清晰的文档字符串

## 示例

查看 `test/test_example.py` 获取完整的使用示例，包括：

- 几何计算测试
- 船舶状态验证测试
- 基于属性的测试示例
- 场景验证集成测试

## 故障排除

### 导入错误

如果遇到导入错误，确保设置了正确的 PYTHONPATH：

```bash
export PYTHONPATH=src:$PYTHONPATH
```

或在运行 pytest 时指定：

```bash
PYTHONPATH=src:$PYTHONPATH pytest
```

### Hypothesis 测试失败

Hypothesis 会自动寻找反例。如果测试失败：

1. 查看失败的反例（Falsifying example）
2. 分析为什么这个输入会导致失败
3. 修复代码或调整测试假设
4. Hypothesis 会记住失败的用例，下次优先测试

### ROS2 依赖问题

如果遇到 ROS2 相关的导入错误，确保：

1. 已安装 ROS2
2. 已 source ROS2 环境：`source /opt/ros/jazzy/setup.bash`
3. 安装了缺失的 Python 包：`pip install lark`

## 贡献

添加新的测试工具或策略时：

1. 在 `strategies.py` 或 `test_helpers.py` 中添加函数
2. 在 `__init__.py` 中导出新函数
3. 在 `test_example.py` 中添加使用示例
4. 更新本 README 文档

## 参考资料

- [pytest 文档](https://docs.pytest.org/)
- [Hypothesis 文档](https://hypothesis.readthedocs.io/)
- [Property-Based Testing 介绍](https://hypothesis.works/articles/what-is-property-based-testing/)
