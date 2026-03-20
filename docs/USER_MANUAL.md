# 海事碰撞避免系统 - 用户手册

## 快速开始

### 安装

```bash
cd /home/waxy/ais_ws
colcon build
source install/setup.bash
```

### 基础使用

```python
from scenario_generator.generator import ScenarioGenerator, HeadOnParams

gen = ScenarioGenerator()
scenario = gen.generate_head_on_scenario(
    HeadOnParams(distance=3.0, speed1=10.0, speed2=10.0)
)
print(f"场景ID: {scenario.scenario_id}")
print(f"船舶数: {len(scenario.ships)}")
```

## 场景类型

### 1. 对遇场景 (Head-on)
两艘船相向航行，航向差接近180°。

```python
from scenario_generator.generator import HeadOnParams
params = HeadOnParams(
    distance=3.0,      # 初始距离（海里）
    speed1=10.0,       # 船1速度（节）
    speed2=10.0        # 船2速度（节）
)
scenario = gen.generate_head_on_scenario(params)
```

### 2. 多船包围场景 (Surrounding)
目标船被多艘船从四周包围。

```python
from scenario_generator.generator import SurroundingParams
params = SurroundingParams(
    num_surrounding=4,           # 包围船数量
    surrounding_distance=2.0,    # 包围距离（海里）
    own_speed=10.0,
    min_speed=8.0,
    max_speed=15.0
)
scenario = gen.generate_surrounding_scenario(params)
```

### 3. 受限水域场景 (Restricted Water)
在狭窄航道中的对遇场景。

```python
from scenario_generator.generator import RestrictedWaterParams
params = RestrictedWaterParams(
    channel_width_nm=0.5,        # 航道宽度（海里）
    channel_length_nm=5.0,       # 航道长度（海里）
    channel_heading=90.0,        # 航道方向（度）
    own_speed=8.0,
    target_speed=8.0
)
scenario = gen.generate_restricted_water_scenario(params)
```

### 4. 恶劣天气场景 (Rough Weather)
在风浪流环境中的对遇场景。

```python
from scenario_generator.generator import RoughWeatherParams
params = RoughWeatherParams(
    wind_speed_ms=15.0,          # 风速（m/s）
    wind_direction=0.0,          # 风向（度）
    current_speed_ms=1.5,        # 流速（m/s）
    current_direction=90.0,      # 流向（度）
    visibility_nm=3.0            # 能见度（海里）
)
scenario = gen.generate_rough_weather_scenario(params)
```

### 5. 能见度不良场景 (Poor Visibility)
低能见度条件下的对遇场景，采用保守的风险阈值。

```python
from scenario_generator.generator import PoorVisibilityParams
params = PoorVisibilityParams(
    visibility_nm=1.0,           # 能见度（海里）
    distance_nm=4.0,             # 初始距离（海里）
    risk_threshold_warning=0.3,  # 预警阈值（低于正常0.5）
    risk_threshold_danger=0.5    # 危险阈值（低于正常0.7）
)
scenario = gen.generate_poor_visibility_scenario(params)
```

## 风险评估

```python
from collision_avoidance.risk_assessment import assess_collision_risk

own_ship = scenario.ships[0]
target_ship = scenario.ships[1]

result = assess_collision_risk(own_ship, target_ship)
print(f"DCPA: {result.dcpa:.2f} nm")
print(f"TCPA: {result.tcpa:.2f} min")
print(f"CRI: {result.cri:.3f}")
print(f"风险等级: {result.risk_level.value}")
```

## 可视化参数发布

```python
from collision_avoidance.visualization_publisher_node import VisualizationPublisherNode

# 在ROS2节点中使用
node = VisualizationPublisherNode()
# 发布到 /visualization/risk_parameters
# 发布到 /visualization/ship_summary
# 发布到 /visualization/alerts
```

## 数据记录和回放

```python
from data_logger.data_models import DataStore, AisRecord, RiskRecord
from data_logger.replay_exporter import ReplayExporter

store = DataStore()
# 添加AIS记录
store.add_ais(AisRecord(timestamp=1000.0, mmsi=123456789, ...))
# 添加风险记录
store.add_risk(RiskRecord(timestamp=1000.0, own_mmsi=123456789, ...))

# 导出回放
exporter = ReplayExporter('/tmp/replay')
replay = exporter.build_replay(store, frame_interval=1.0)
exporter.export_json(replay, 'scenario_replay.json')
```

## 配置热加载

```python
from collision_avoidance.config_watcher import ConfigWatcher

def on_config_reload(config):
    print(f"配置已重载: {config}")

watcher = ConfigWatcher(
    'config.yaml',
    on_reload=on_config_reload,
    poll_interval=1.0,
    auto_start=True
)

# 获取配置
threshold = watcher.get('collision_threshold', default=0.5)
```

## 常见问题

**Q: 如何自定义避碰算法？**
A: 继承 `AvoidanceAlgorithm` 类并实现 `compute_strategy()` 方法，然后注册到全局注册表。

**Q: 支持哪些场景类型？**
A: 对遇、交叉、追越、多船、紧急、包围、受限水域、恶劣天气、能见度不良。

**Q: 如何调整风险阈值？**
A: 在场景参数中设置 `risk_threshold_warning` 和 `risk_threshold_danger`。

## 更多信息

- API文档：见 `docs/api.md`
- 开发者指南：见 `docs/developer.md`
- 示例场景：见 `examples/scenarios/`
