# 场景加载器使用示例

## 快速开始

### 1. 加载预定义场景

```python
from scenario_generator import ScenarioLoader

# 创建加载器
loader = ScenarioLoader()

# 列出所有可用场景
scenarios = loader.list_scenarios()
print("可用场景:", scenarios)

# 加载基础对遇场景
scenario = loader.load_from_yaml('head_on_basic.yaml')

# 查看场景信息
print(f"场景ID: {scenario.scenario_id}")
print(f"场景类型: {scenario.scenario_type}")
print(f"船舶数量: {len(scenario.ships)}")
print(f"描述: {scenario.description}")
```

### 2. 使用场景配置

```python
# 获取船舶状态
for ship in scenario.ships:
    print(f"船舶 {ship.mmsi}:")
    print(f"  位置: ({ship.latitude}, {ship.longitude})")
    print(f"  航向: {ship.heading}°")
    print(f"  速度: {ship.sog} 节")

# 获取环境配置
env = scenario.environment
print(f"天气: {env.weather_condition.value}")
print(f"能见度: {env.visibility.value}")
print(f"风速: {env.wind_speed} m/s")
```

### 3. 转换为字典格式

```python
# 转换为字典（用于序列化或传递给其他模块）
scenario_dict = scenario.to_dict()

# 保存为JSON
import json
with open('scenario.json', 'w') as f:
    json.dump(scenario_dict, f, indent=2)
```

### 4. 程序化生成场景

```python
from scenario_generator import ScenarioGenerator, HeadOnParams

# 创建生成器
generator = ScenarioGenerator()

# 自定义参数
params = HeadOnParams(
    distance=15.0,      # 15海里
    speed1=20.0,        # 20节
    speed2=18.0,        # 18节
    base_latitude=31.0,
    base_longitude=121.0
)

# 生成场景
scenario = generator.generate_head_on_scenario(params)
```

### 5. 保存自定义场景

```python
# 保存生成的场景为YAML文件
loader.save_to_yaml(scenario, 'my_custom_scenario.yaml')
```

## 集成到测试框架

```python
import pytest
from scenario_generator import ScenarioLoader

@pytest.fixture
def scenario_loader():
    return ScenarioLoader()

@pytest.fixture(params=[
    'head_on_basic.yaml',
    'head_on_high_speed.yaml',
    'head_on_close.yaml'
])
def test_scenario(request, scenario_loader):
    return scenario_loader.load_from_yaml(request.param)

def test_collision_avoidance(test_scenario):
    """测试避碰算法"""
    # 使用加载的场景进行测试
    result = run_collision_avoidance_test(test_scenario)
    assert result.collision_avoided == True
    assert result.min_distance >= test_scenario.success_criteria['min_distance']
```

## 批量运行场景

```python
from scenario_generator import ScenarioLoader

def run_all_scenarios():
    loader = ScenarioLoader()
    results = {}
    
    for scenario_file in loader.list_scenarios():
        print(f"\n运行场景: {scenario_file}")
        scenario = loader.load_from_yaml(scenario_file)
        
        # 运行你的避碰算法
        result = run_collision_avoidance(scenario)
        results[scenario_file] = result
        
        print(f"  结果: {'通过' if result.success else '失败'}")
    
    return results

# 执行
results = run_all_scenarios()
```

## 自定义场景目录

```python
# 使用自定义场景目录
loader = ScenarioLoader(scenarios_dir='/path/to/my/scenarios')

# 加载场景
scenario = loader.load_from_yaml('my_scenario.yaml')
```

## 注意事项

1. 所有距离单位为海里（nautical miles）
2. 所有速度单位为节（knots）
3. 角度单位为度（degrees），范围0-360
4. 时间单位为秒（seconds）
5. MMSI必须是9位数字

## 常见问题

**Q: 如何修改现有场景？**
A: 直接编辑YAML文件，或者加载后修改对象属性再保存。

**Q: 场景文件放在哪里？**
A: 默认在 `src/scenario_generator/config/scenarios/` 目录。

**Q: 如何添加新的场景类型？**
A: 等待其他场景生成器实现后（交叉、追越等），按照相同格式创建YAML文件。
