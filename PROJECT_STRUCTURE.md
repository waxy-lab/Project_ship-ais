# 项目结构说明

## 概述

本项目是一个基于ROS2的船舶避碰仿真测试系统，采用模块化设计，包含三个核心包。

## 目录结构

```
maritime-collision-avoidance/
├── src/
│   ├── scenario_generator/          # 场景生成器包
│   │   ├── scenario_generator/      # 主模块
│   │   │   ├── __init__.py
│   │   │   ├── models.py           # 数据模型（已实现）
│   │   │   ├── config_loader.py    # 配置加载器（已实现）
│   │   │   └── generator.py        # 场景生成器（待实现）
│   │   ├── test/                   # 测试目录
│   │   │   ├── __init__.py
│   │   │   └── test_models.py      # 模型测试
│   │   ├── config/                 # 配置文件目录
│   │   ├── resource/               # ROS2资源目录
│   │   ├── setup.py                # Python包配置
│   │   ├── setup.cfg               # 安装配置
│   │   └── package.xml             # ROS2包配置
│   │
│   ├── collision_avoidance/         # 避碰决策包
│   │   ├── collision_avoidance/     # 主模块
│   │   │   ├── __init__.py
│   │   │   ├── risk_assessment.py  # 风险评估（待实现）
│   │   │   ├── rules_engine.py     # COLREGS规则引擎（待实现）
│   │   │   ├── path_planning.py    # 路径规划（待实现）
│   │   │   └── ca_node.py          # ROS2节点（待实现）
│   │   ├── test/                   # 测试目录
│   │   ├── config/                 # 配置文件目录
│   │   ├── resource/               # ROS2资源目录
│   │   ├── setup.py
│   │   ├── setup.cfg
│   │   └── package.xml
│   │
│   ├── test_framework/              # 测试评估框架包
│   │   ├── test_framework/          # 主模块
│   │   │   ├── __init__.py
│   │   │   ├── test_runner.py      # 测试运行器（待实现）
│   │   │   ├── metrics.py          # 性能指标（待实现）
│   │   │   └── data_logger.py      # 数据记录器（待实现）
│   │   ├── test/                   # 测试目录
│   │   ├── config/                 # 配置文件目录
│   │   ├── resource/               # ROS2资源目录
│   │   ├── setup.py
│   │   ├── setup.cfg
│   │   └── package.xml
│   │
│   ├── ais_simulator/               # AIS模拟器（现有）
│   └── ais_to_mqtt/                 # AIS到MQTT桥接（现有）
│
├── .kiro/
│   └── specs/
│       └── maritime-collision-avoidance/
│           ├── requirements.md      # 需求文档
│           ├── design.md           # 设计文档
│           └── tasks.md            # 任务列表
│
├── config/                          # 全局配置
│   └── system_config.yaml
│
├── install/                         # 构建输出（colcon）
├── build/                          # 构建临时文件
├── log/                            # 构建日志
│
├── reorganize_project.py           # 项目重组脚本
├── MIGRATION_SUMMARY.md            # 迁移摘要
├── PROJECT_STRUCTURE.md            # 本文档
├── pytest.ini                      # pytest配置
├── requirements.txt                # Python依赖
└── README.md                       # 项目说明
```

## 核心包说明

### 1. scenario_generator（场景生成器）

**功能**: 自动生成各种船舶相遇场景

**主要模块**:
- `models.py`: 定义数据模型（ShipState, ScenarioConfig, EnvironmentConfig等）
- `config_loader.py`: 从YAML文件加载场景配置
- `generator.py`: 实现场景生成算法（对遇、交叉、追越等）

**依赖**: rclpy, std_msgs

**状态**: 
- ✅ 包结构完整
- ✅ 数据模型已实现
- ✅ 配置加载器已实现
- ⏳ 场景生成器待实现

### 2. collision_avoidance（避碰决策模块）

**功能**: 基于COLREGS规则的智能避碰算法

**主要模块**:
- `risk_assessment.py`: 计算DCPA、TCPA、CRI等风险指标
- `rules_engine.py`: 实现COLREGS规则判定和应用
- `path_planning.py`: 生成和评估避让路径
- `ca_node.py`: ROS2节点，订阅AIS数据，发布控制指令

**依赖**: rclpy, std_msgs, scenario_generator

**状态**: 
- ✅ 包结构完整
- ⏳ 所有模块待实现

### 3. test_framework（测试评估框架）

**功能**: 批量运行测试场景，统计性能指标

**主要模块**:
- `test_runner.py`: 批量运行场景，收集数据
- `metrics.py`: 计算性能指标（成功率、距离、合规性等）
- `data_logger.py`: 记录仿真数据到数据库或文件

**依赖**: rclpy, std_msgs, scenario_generator, collision_avoidance

**状态**: 
- ✅ 包结构完整
- ⏳ 所有模块待实现

## 包依赖关系

```
test_framework
    ├── collision_avoidance
    │   └── scenario_generator
    └── scenario_generator

ais_simulator (独立)
ais_to_mqtt (独立)
```

## 构建和测试

### 构建所有包

```bash
# 构建新的三个包
colcon build --packages-select scenario_generator collision_avoidance test_framework

# 构建所有包
colcon build

# 加载环境
source install/setup.bash
```

### 运行测试

```bash
# 使用colcon运行测试
colcon test --packages-select scenario_generator collision_avoidance test_framework

# 查看测试结果
colcon test-result --verbose

# 使用pytest直接运行
pytest src/scenario_generator/test/
pytest src/collision_avoidance/test/
pytest src/test_framework/test/
```

### 清理构建

```bash
rm -rf build/ install/ log/
```

## 开发工作流

1. **查看任务列表**: `.kiro/specs/maritime-collision-avoidance/tasks.md`
2. **选择任务**: 按照任务编号顺序实现
3. **编写代码**: 在对应的包中实现功能
4. **编写测试**: 在test目录中添加单元测试和属性测试
5. **构建验证**: `colcon build --packages-select <package_name>`
6. **运行测试**: `pytest src/<package_name>/test/`
7. **提交代码**: 确保所有测试通过

## 测试策略

本项目采用双重测试策略：

1. **单元测试（Unit Tests）**: 使用pytest，测试具体功能和边界情况
2. **属性测试（Property-Based Tests）**: 使用Hypothesis，验证通用属性

每个属性测试应：
- 运行至少100次迭代
- 标注对应的设计属性编号
- 格式：`# Feature: maritime-collision-avoidance, Property N: [property text]`

## 代码规范

- Python代码遵循PEP 8规范
- 使用类型提示（Type Hints）
- 编写详细的文档字符串（Docstrings）
- 所有公共API必须有文档
- 测试覆盖率目标：>80%

## 下一步工作

根据tasks.md的任务顺序：

1. ✅ **Task 1.1**: 项目结构重组（已完成）
2. ⏳ **Task 1.2**: 配置pytest和Hypothesis测试框架
3. ⏳ **Task 2.x**: 实现场景生成器核心功能
4. ⏳ **Task 4.x**: 实现碰撞风险评估模块
5. ⏳ **Task 5.x**: 实现COLREGS规则引擎

详细任务列表请参考：`.kiro/specs/maritime-collision-avoidance/tasks.md`

## 参考文档

- **需求文档**: `.kiro/specs/maritime-collision-avoidance/requirements.md`
- **设计文档**: `.kiro/specs/maritime-collision-avoidance/design.md`
- **任务列表**: `.kiro/specs/maritime-collision-avoidance/tasks.md`
- **迁移摘要**: `MIGRATION_SUMMARY.md`

## 联系方式

如有问题，请参考设计文档或联系开发团队。
