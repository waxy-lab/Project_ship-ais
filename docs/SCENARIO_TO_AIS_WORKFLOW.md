# 场景生成器 → AIS模拟器 工作流程

## 概述

场景生成器负责创建标准化的测试场景，AIS模拟器负责实时模拟船舶运动。通过转换工具，可以将场景配置无缝应用到AIS模拟器。

## 工作流程

```
┌─────────────────────┐
│  场景生成器          │
│  (Scenario Generator)│
│                     │
│  - 生成标准场景      │
│  - 定义初始状态      │
│  - 环境配置         │
└──────────┬──────────┘
           │
           │ YAML配置
           ↓
┌─────────────────────┐
│  转换工具            │
│  (Converter)        │
└──────────┬──────────┘
           │
           │ ships_config.yaml
           ↓
┌─────────────────────┐
│  AIS模拟器          │
│  (AIS Simulator)    │
│                     │
│  - 实时运动模拟      │
│  - 输出AIS数据流     │
│  - 避障行为         │
└─────────────────────┘
```

## 使用步骤

### 1. 选择或创建场景

**查看可用场景：**
```bash
python3 convert_scenario_to_ais.py --list
```

**使用预定义场景：**
- `head_on_basic.yaml` - 基础对遇（10海里，15节）
- `head_on_high_speed.yaml` - 高速对遇（8海里，25/22节）
- `head_on_close.yaml` - 近距离对遇（5海里）
- `head_on_asymmetric.yaml` - 速度不对称（快船vs慢船）
- `head_on_poor_visibility.yaml` - 能见度不良

**或创建自定义场景：**
```python
from scenario_generator import ScenarioGenerator, HeadOnParams

generator = ScenarioGenerator()
params = HeadOnParams(
    distance=12.0,
    speed1=20.0,
    speed2=18.0,
    base_latitude=31.0,
    base_longitude=121.0
)
scenario = generator.generate_head_on_scenario(params)

# 保存为场景文件
from scenario_generator import ScenarioLoader
loader = ScenarioLoader()
loader.save_to_yaml(scenario, 'my_scenario.yaml')
```

### 2. 转换场景为AIS配置

**基本用法：**
```bash
# 转换场景（自动覆盖 src/ais_simulator/config/ships_config.yaml）
python3 convert_scenario_to_ais.py head_on_basic.yaml
```

**指定输出路径：**
```bash
python3 convert_scenario_to_ais.py head_on_basic.yaml -o custom_config.yaml
```

**不添加注释：**
```bash
python3 convert_scenario_to_ais.py head_on_basic.yaml --no-comments
```

### 3. 启动AIS模拟器

```bash
# 使用默认配置
ros2 run ais_simulator ais_sim_node

# 或指定配置文件
ros2 run ais_simulator ais_sim_node --ros-args \
  -p ships_configs:=/path/to/custom_config.yaml
```

### 4. 验证运行

**查看AIS数据输出：**
```bash
# 监听串口数据
cat /dev/pts/2

# 或使用ROS2话题（如果发布了）
ros2 topic echo /ais/ship_states
```

## 配置格式对比

### 场景生成器格式 (ScenarioConfig)

```yaml
scenario_type: head_on
description: "基础对遇场景"
parameters:
  distance: 10.0
  speed1: 15.0
  speed2: 15.0
  base_latitude: 30.0
  base_longitude: 120.0
environment:
  weather_condition: calm
  visibility: good
duration: 600.0
```

### AIS模拟器格式 (ships_config.yaml)

```yaml
simulated_ships:
  - mmsi: 123456789
    latitude: 30.0
    longitude: 119.91667
    heading: 90.0
    sog: 15.0
    rot: 0.0
  - mmsi: 987654321
    latitude: 30.0
    longitude: 120.08333
    heading: 270.0
    sog: 15.0
    rot: 0.0
```

## 批量测试场景

```bash
#!/bin/bash
# 批量运行所有场景

SCENARIOS=(
    "head_on_basic.yaml"
    "head_on_high_speed.yaml"
    "head_on_close.yaml"
)

for scenario in "${SCENARIOS[@]}"; do
    echo "=========================================="
    echo "测试场景: $scenario"
    echo "=========================================="
    
    # 转换配置
    python3 convert_scenario_to_ais.py "$scenario"
    
    # 启动AIS模拟器（后台运行）
    ros2 run ais_simulator ais_sim_node &
    AIS_PID=$!
    
    # 运行10分钟
    sleep 600
    
    # 停止模拟器
    kill $AIS_PID
    
    echo "场景 $scenario 测试完成"
    echo ""
done
```

## Python API 使用

```python
from scenario_generator import (
    ScenarioLoader,
    AISConfigConverter
)

# 方法1：使用便捷函数
from scenario_generator import convert_scenario_to_ais
output_path = convert_scenario_to_ais('head_on_basic.yaml')
print(f"配置已保存到: {output_path}")

# 方法2：手动转换
loader = ScenarioLoader()
scenario = loader.load_from_yaml('head_on_basic.yaml')

converter = AISConfigConverter()
converter.save_ais_config(
    scenario,
    'src/ais_simulator/config/ships_config.yaml'
)
```

## 注意事项

1. **坐标系统**：场景生成器使用WGS84坐标系（经纬度）
2. **单位**：距离=海里，速度=节，角度=度
3. **航点**：场景生成器目前不生成航点，需要手动添加
4. **环境参数**：AIS模拟器目前不使用环境配置（风、流等）

## 未来扩展

待其他场景类型实现后，可以转换：
- 交叉相遇场景
- 追越场景
- 多船复杂场景
- 紧急避让场景

## 故障排除

**问题：转换后AIS模拟器无法启动**
- 检查MMSI是否为9位数字
- 检查坐标是否在有效范围内
- 检查速度是否为正数

**问题：船舶位置不在预期区域**
- 场景生成器使用的基准坐标可能与你的地图不匹配
- 修改场景YAML中的 `base_latitude` 和 `base_longitude`

**问题：需要添加航点**
- 转换后手动编辑 `ships_config.yaml`
- 或在场景生成时通过 `ShipState` 的 `waypoints` 参数指定
