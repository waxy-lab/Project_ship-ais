# 示例场景库

本目录包含10个典型场景配置，涵盖各种相遇类型和环境条件。

## 场景列表

### 1. 基础对遇 (head_on_basic.yaml)
- 类型：对遇
- 距离：3.0 nm
- 速度：10 kn / 10 kn
- 环境：平静

### 2. 高速对遇 (head_on_high_speed.yaml)
- 类型：对遇
- 距离：5.0 nm
- 速度：15 kn / 15 kn
- 环境：平静

### 3. 近距离对遇 (head_on_close.yaml)
- 类型：对遇
- 距离：1.0 nm
- 速度：10 kn / 10 kn
- 环境：平静

### 4. 不对称对遇 (head_on_asymmetric.yaml)
- 类型：对遇
- 距离：3.0 nm
- 速度：12 kn / 8 kn
- 环境：平静

### 5. 能见度不良对遇 (head_on_poor_visibility.yaml)
- 类型：对遇
- 距离：3.0 nm
- 速度：8 kn / 8 kn
- 环境：能见度 1.0 nm

### 6. 多船包围 (surrounding_4ships.yaml)
- 类型：多船包围
- 包围船数：4
- 包围距离：2.0 nm
- 环境：平静

### 7. 受限水域 (restricted_channel.yaml)
- 类型：受限水域
- 航道宽度：0.5 nm
- 航道长度：5.0 nm
- 环境：平静

### 8. 恶劣天气 (rough_weather.yaml)
- 类型：恶劣天气
- 风速：15 m/s
- 流速：1.5 m/s
- 能见度：3.0 nm

### 9. 极端恶劣天气 (extreme_weather.yaml)
- 类型：恶劣天气
- 风速：25 m/s
- 流速：2.5 m/s
- 能见度：1.0 nm

### 10. 复杂多船 (complex_multi_ship.yaml)
- 类型：多船
- 船舶数：5
- 区域大小：10.0 nm
- 环境：中等天气

## 使用方法

```python
from scenario_generator.generator import ScenarioGenerator
import yaml

gen = ScenarioGenerator()

# 加载配置
with open('examples/scenarios/head_on_basic.yaml') as f:
    config = yaml.safe_load(f)

# 生成场景
scenario = gen.generate_head_on_scenario(
    HeadOnParams(**config['parameters'])
)
```

## 场景参数说明

- `distance_nm`: 初始距离（海里）
- `speed1`, `speed2`: 船舶速度（节）
- `wind_speed_ms`: 风速（米/秒）
- `visibility_nm`: 能见度（海里）
- `num_ships`: 船舶数量
- `channel_width_nm`: 航道宽度（海里）

## 扩展

可根据需要创建新的场景配置文件，遵循相同的YAML格式。
