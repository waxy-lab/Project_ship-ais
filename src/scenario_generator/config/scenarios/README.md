# 预定义场景库

本目录包含预配置的测试场景，可直接加载使用。

## 场景列表

### 对遇场景 (Head-On)

1. **head_on_basic.yaml** - 基础对遇场景
   - 距离：10海里
   - 速度：15节 vs 15节
   - 环境：平静、能见度良好
   - 用途：基础功能测试

2. **head_on_high_speed.yaml** - 高速对遇场景
   - 距离：8海里
   - 速度：25节 vs 22节
   - 环境：中等风浪
   - 用途：测试高速场景下的快速决策

3. **head_on_close.yaml** - 近距离对遇场景
   - 距离：5海里
   - 速度：18节 vs 16节
   - 环境：能见度中等
   - 用途：测试紧急避让能力

4. **head_on_asymmetric.yaml** - 不对称速度对遇
   - 距离：12海里
   - 速度：25节 vs 10节（快船vs慢船）
   - 环境：平静
   - 用途：测试速度差异大的场景

5. **head_on_poor_visibility.yaml** - 能见度不良对遇
   - 距离：10海里
   - 速度：12节 vs 12节（雾天减速）
   - 环境：雾天、能见度不良
   - 用途：测试恶劣天气下的避让

## 使用方法

```python
from scenario_generator.scenario_loader import ScenarioLoader

# 加载预定义场景
loader = ScenarioLoader()
scenario = loader.load_from_yaml('head_on_basic.yaml')

# 运行场景
# ... 你的测试代码
```

## 场景文件格式

每个YAML文件包含以下字段：
- `scenario_type`: 场景类型（head_on, crossing, overtaking等）
- `description`: 场景描述
- `parameters`: 场景参数（距离、速度、位置等）
- `environment`: 环境配置（天气、能见度、风浪等）
- `duration`: 场景持续时间（秒）
- `success_criteria`: 成功判定标准

## 扩展场景库

待其他场景生成器实现后，将添加：
- 交叉相遇场景 (crossing_*.yaml)
- 追越场景 (overtaking_*.yaml)
- 多船场景 (multi_ship_*.yaml)
- 紧急场景 (emergency_*.yaml)
