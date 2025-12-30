# Requirements Document

## Introduction

本文档定义了"水路仿真测试框架与智能避碰系统"的需求规范。该系统旨在构建一个完整的船舶避碰仿真测试平台，能够模拟多种复杂的船舶相遇场景，并基于国际海上避碰规则（COLREGS）实现智能避碰算法。

## Glossary

- **System**: 水路仿真测试框架与智能避碰系统
- **AIS**: 船舶自动识别系统 (Automatic Identification System)
- **COLREGS**: 国际海上避碰规则 (International Regulations for Preventing Collisions at Sea)
- **Simulator**: AIS数据模拟器，负责生成船舶位置和状态数据
- **Collision_Avoidance_Module**: 避碰决策模块，基于COLREGS规则计算避碰策略
- **Scenario_Generator**: 场景生成器，用于创建各种船舶相遇场景
- **Test_Framework**: 测试框架，用于评估避碰算法性能
- **DCPA**: 最近会遇距离 (Distance at Closest Point of Approach)
- **TCPA**: 到达最近会遇点时间 (Time to Closest Point of Approach)
- **CRI**: 碰撞风险指数 (Collision Risk Index)

## Requirements

### Requirement 1: 场景仿真能力

**User Story:** 作为测试工程师，我希望系统能够模拟多种船舶相遇场景，以便全面测试避碰算法的有效性。

#### Acceptance Criteria

1. WHEN 用户配置对遇场景参数，THE Scenario_Generator SHALL 生成两艘船舶相向航行的初始状态
2. WHEN 用户配置交叉相遇场景参数，THE Scenario_Generator SHALL 生成两艘船舶交叉航行的初始状态
3. WHEN 用户配置追越场景参数，THE Scenario_Generator SHALL 生成一艘船舶追赶另一艘船舶的初始状态
4. WHEN 用户配置多船复杂场景参数，THE Scenario_Generator SHALL 生成3艘及以上船舶的复杂相遇状态
5. WHEN 场景开始运行，THE Simulator SHALL 以1Hz频率发送每艘船舶的AIS数据

### Requirement 2: 极端场景测试

**User Story:** 作为安全工程师，我希望系统能够模拟极端和危险场景，以验证避碰算法在压力情况下的表现。

#### Acceptance Criteria

1. WHEN 用户配置紧急避让场景，THE Scenario_Generator SHALL 生成DCPA小于0.5海里且TCPA小于5分钟的危险相遇状态
2. WHEN 用户配置多船包围场景，THE Scenario_Generator SHALL 生成目标船被多艘船舶从不同方向包围的状态
3. WHEN 用户配置受限水域场景，THE Scenario_Generator SHALL 生成狭窄航道或港口水域的约束条件
4. WHEN 用户配置恶劣天气场景，THE Scenario_Generator SHALL 在船舶运动模型中加入风浪干扰参数
5. WHEN 用户配置能见度不良场景，THE Scenario_Generator SHALL 调整碰撞风险阈值和避让时机参数

### Requirement 3: COLREGS规则实现

**User Story:** 作为算法工程师，我希望系统严格遵循国际海上避碰规则，以确保避碰决策的合法性和安全性。

#### Acceptance Criteria

1. WHEN 检测到对遇局面（Rule 14），THE Collision_Avoidance_Module SHALL 指令本船向右转向
2. WHEN 检测到交叉相遇且本船为让路船（Rule 15），THE Collision_Avoidance_Module SHALL 采取明显避让行动
3. WHEN 检测到交叉相遇且本船为直航船（Rule 17），THE Collision_Avoidance_Module SHALL 保持航向和航速，直到让路船未采取行动
4. WHEN 检测到追越局面（Rule 13），THE Collision_Avoidance_Module SHALL 保持清晰避让被追越船
5. WHEN 在受限水域航行（Rule 9），THE Collision_Avoidance_Module SHALL 优先考虑航道规则和吃水限制
6. WHEN 能见度不良（Rule 19），THE Collision_Avoidance_Module SHALL 采取更保守的避让策略

### Requirement 4: 碰撞风险评估

**User Story:** 作为系统操作员，我希望系统能够实时评估碰撞风险，以便及时做出避让决策。

#### Acceptance Criteria

1. WHEN 接收到他船AIS数据，THE Collision_Avoidance_Module SHALL 计算DCPA和TCPA
2. WHEN DCPA和TCPA计算完成，THE Collision_Avoidance_Module SHALL 基于多因素模型计算CRI值
3. WHEN CRI超过预警阈值，THE System SHALL 发出碰撞预警信号
4. WHEN CRI超过危险阈值，THE Collision_Avoidance_Module SHALL 触发自动避让决策
5. WHEN 多艘船舶同时存在碰撞风险，THE Collision_Avoidance_Module SHALL 按照风险优先级排序处理

### Requirement 5: 避让路径规划

**User Story:** 作为导航系统，我希望系统能够规划安全且高效的避让路径，以最小代价完成避碰。

#### Acceptance Criteria

1. WHEN 触发避让决策，THE Collision_Avoidance_Module SHALL 计算多条候选避让路径
2. WHEN 评估候选路径，THE Collision_Avoidance_Module SHALL 考虑航向改变量、航程增加量和时间成本
3. WHEN 评估候选路径，THE Collision_Avoidance_Module SHALL 验证路径不会引发新的碰撞风险
4. WHEN 评估候选路径，THE Collision_Avoidance_Module SHALL 确保路径符合COLREGS规则要求
5. WHEN 选择最优路径，THE Collision_Avoidance_Module SHALL 输出航向和航速调整指令
6. WHEN 避让完成后，THE Collision_Avoidance_Module SHALL 规划返回原航线的路径

### Requirement 6: 算法性能评估

**User Story:** 作为研究人员，我希望系统能够量化评估避碰算法的性能，以便进行算法优化和对比。

#### Acceptance Criteria

1. WHEN 场景仿真结束，THE Test_Framework SHALL 统计避碰成功率
2. WHEN 场景仿真结束，THE Test_Framework SHALL 计算平均避让距离和最小会遇距离
3. WHEN 场景仿真结束，THE Test_Framework SHALL 统计航向改变次数和平均改变角度
4. WHEN 场景仿真结束，THE Test_Framework SHALL 计算航程增加百分比和时间延误
5. WHEN 场景仿真结束，THE Test_Framework SHALL 评估COLREGS规则遵守率
6. WHEN 运行批量测试，THE Test_Framework SHALL 生成性能对比报告和可视化图表

### Requirement 7: 可视化与监控

**User Story:** 作为系统用户，我希望能够实时观察船舶运动和避碰过程，以便理解系统行为和调试问题。

#### Acceptance Criteria

1. WHEN 场景运行时，THE System SHALL 在地图上实时显示所有船舶的位置和航向
2. WHEN 场景运行时，THE System SHALL 可视化显示船舶的预测轨迹和碰撞风险区域
3. WHEN 触发避让决策，THE System SHALL 高亮显示避让船舶和规划路径
4. WHEN 场景运行时，THE System SHALL 实时显示关键参数（DCPA、TCPA、CRI）
5. WHEN 场景结束后，THE System SHALL 支持回放功能以复现整个避碰过程

### Requirement 8: 配置与扩展性

**User Story:** 作为开发者，我希望系统具有良好的配置能力和扩展性，以便快速调整参数和添加新功能。

#### Acceptance Criteria

1. WHEN 用户修改配置文件，THE System SHALL 支持热加载场景参数和算法参数
2. WHEN 用户需要自定义场景，THE System SHALL 提供场景配置文件模板和文档
3. WHEN 用户需要扩展避碰算法，THE System SHALL 提供清晰的算法接口和插件机制
4. WHEN 用户需要集成外部数据，THE System SHALL 支持标准AIS数据格式和ROS2消息接口
5. WHERE 用户需要分布式部署，THE System SHALL 支持多节点协同仿真

### Requirement 9: 数据记录与分析

**User Story:** 作为数据分析师，我希望系统能够完整记录仿真数据，以便进行离线分析和算法改进。

#### Acceptance Criteria

1. WHEN 场景运行时，THE System SHALL 记录所有船舶的AIS数据到时序数据库
2. WHEN 场景运行时，THE System SHALL 记录所有避让决策和控制指令
3. WHEN 场景运行时，THE System SHALL 记录碰撞风险评估的中间计算结果
4. WHEN 场景结束后，THE System SHALL 将数据导出为标准格式（CSV、JSON、ROS Bag）
5. WHEN 用户需要分析数据，THE System SHALL 提供数据查询和统计分析工具

### Requirement 10: 系统集成与部署

**User Story:** 作为系统管理员，我希望系统易于部署和集成到现有环境中，以降低使用门槛。

#### Acceptance Criteria

1. WHEN 用户首次部署，THE System SHALL 提供一键安装脚本和依赖管理
2. WHEN 系统启动，THE System SHALL 自动检查依赖项和配置文件完整性
3. WHEN 系统运行，THE System SHALL 提供健康检查接口和日志监控
4. WHEN 系统出错，THE System SHALL 提供详细的错误信息和恢复建议
5. WHERE 用户需要容器化部署，THE System SHALL 提供Docker镜像和编排配置
