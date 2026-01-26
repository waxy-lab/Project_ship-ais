# Implementation Plan: 水路仿真测试框架与智能避碰系统

## Overview

本实现计划将系统开发分为4个主要阶段，每个阶段包含多个可执行的编码任务。任务按照依赖关系排序，确保每一步都能在前一步的基础上进行。标记为`*`的任务为可选任务，可以在MVP阶段跳过。

## 当前实现状态总结

已完成的核心模块：
- ✅ 项目结构：scenario_generator, collision_avoidance, test_framework 包已创建
- ✅ 测试框架：pytest + Hypothesis 已配置，包含策略生成器和测试工具
- ✅ 数据模型：ShipState, EnvironmentConfig, ScenarioConfig 已实现并包含完整验证
- ✅ 对遇场景生成器：已实现并通过属性测试验证
- ✅ AIS模拟器：已有基础实现，包含船舶运动模型和简单避碰逻辑

待实现的核心功能：
- ❌ 其他场景生成器（交叉、追越、多船、紧急）
- ❌ 碰撞风险评估模块（DCPA/TCPA/CRI计算）
- ❌ COLREGS规则引擎
- ❌ 路径规划模块
- ❌ 避碰决策ROS2节点
- ❌ 测试评估框架
- ❌ 数据记录模块

## Tasks

- [x] 1. 项目结构重组和基础设施搭建
  - [x] 1.1 编写项目结构重组脚本
  - [x] 1.2 配置pytest和Hypothesis测试框架

- [x] 2. 实现场景生成器核心功能（对遇场景）
  - [x] 2.1 创建场景配置数据模型
  - [x] 2.2 编写场景配置数据模型的单元测试
  - [x] 2.3 实现对遇场景生成器
  - [x] 2.4 编写对遇场景生成的属性测试

- [ ] 3. 实现其他场景生成器
  - [x] 3.1 实现交叉相遇场景生成器
    - 实现 generate_crossing_scenario() 方法
    - 计算交叉角度和初始位置，确保相对方位在5-112.5度范围
    - _Requirements: 1.2_

  - [x] 3.2 编写交叉场景生成的属性测试
    - **Property 2: 交叉场景的角度约束**
    - **Validates: Requirements 1.2**
    - _Requirements: 1.2_

  - [x] 3.3 实现追越场景生成器
    - 实现 generate_overtaking_scenario() 方法
    - 确保追越船在后方22.5度扇形区域且速度更快
    - _Requirements: 1.3_

  - [x] 3.4 编写追越场景生成的属性测试
    - **Property 3: 追越场景的速度和位置关系**
    - **Validates: Requirements 1.3**
    - _Requirements: 1.3_

  - [ ] 3.5 实现多船复杂场景生成器
    - 实现 generate_multi_ship_scenario() 方法
    - 支持3艘以上船舶的场景，随机分布在指定区域
    - _Requirements: 1.4_

  - [ ]* 3.6 编写多船场景生成的单元测试
    - 测试船舶数量和初始状态
    - _Requirements: 1.4_

- [ ] 4. 实现极端场景生成器
  - [ ] 4.1 实现紧急避让场景生成器
    - 实现 generate_emergency_scenario() 方法
    - 生成DCPA<0.5海里且TCPA<5分钟的场景
    - _Requirements: 2.1_

  - [ ]* 4.2 编写紧急场景的属性测试
    - **Property 5: 危险场景的风险参数**
    - **Validates: Requirements 2.1**
    - _Requirements: 2.1_

- [ ] 5. Checkpoint - 场景生成器验证
  - 运行所有场景生成器测试
  - 手动验证生成的场景配置文件
  - 确保所有测试通过，询问用户是否有问题

- [ ] 6. 实现碰撞风险评估模块
  - [ ] 6.1 创建风险评估模块文件结构
    - 在 collision_avoidance 包中创建 risk_assessment.py
    - 定义风险评估相关的数据类和常量
    - _Requirements: 4.1-4.4_

  - [ ] 6.2 实现DCPA/TCPA计算函数
    - 实现 calculate_dcpa_tcpa() 函数
    - 使用相对运动矢量法，处理边界情况
    - _Requirements: 4.1_

  - [ ]* 6.3 编写DCPA/TCPA计算的属性测试
    - **Property 6: DCPA/TCPA计算的对称性**
    - **Validates: Requirements 4.1**
    - 验证从两个视角计算结果一致
    - _Requirements: 4.1_

  - [ ]* 6.4 编写DCPA/TCPA计算的单元测试
    - 测试平行航行、相向航行等特殊情况
    - 测试边界情况（静止船舶、相同位置等）
    - _Requirements: 4.1_

  - [ ] 6.5 实现CRI（碰撞风险指数）计算函数
    - 实现 calculate_cri() 函数
    - 综合考虑距离、时间、方位、速度比等因素
    - _Requirements: 4.2_

  - [ ]* 6.6 编写CRI计算的属性测试
    - **Property 10: CRI值的单调性**
    - **Validates: Requirements 4.2**
    - 验证DCPA/TCPA减小时CRI增加
    - _Requirements: 4.2_

  - [ ] 6.7 实现风险阈值判定逻辑
    - 实现预警阈值和危险阈值判定
    - 定义 RiskLevel 枚举和判定函数
    - _Requirements: 4.3, 4.4_

  - [ ]* 6.8 编写风险阈值触发的属性测试
    - **Property 11: 风险阈值触发的一致性**
    - **Validates: Requirements 4.3, 4.4**
    - _Requirements: 4.3, 4.4_

- [ ] 7. 实现COLREGS规则引擎
  - [ ] 7.1 创建规则引擎模块文件
    - 在 collision_avoidance 包中创建 rules_engine.py
    - 定义 EncounterType 枚举和相关数据结构
    - _Requirements: 3.1-3.6_

  - [ ] 7.2 实现相遇类型判定函数
    - 实现 determine_encounter_type() 函数
    - 判断对遇、交叉、追越等类型
    - _Requirements: 3.1-3.4_

  - [ ]* 7.3 编写相遇类型判定的单元测试
    - 测试各种相遇类型的判定
    - 测试边界情况
    - _Requirements: 3.1-3.4_

  - [ ] 7.4 实现COLREGS规则应用函数
    - 实现 apply_colregs_rule() 函数
    - 根据相遇类型返回避让动作
    - _Requirements: 3.1-3.6_

  - [ ]* 7.5 编写对遇规则的属性测试
    - **Property 7: 对遇规则的转向方向**
    - **Validates: Requirements 3.1**
    - 验证对遇时向右转向
    - _Requirements: 3.1_

  - [ ]* 7.6 编写让路船避让的属性测试
    - **Property 8: 让路船的明显避让**
    - **Validates: Requirements 3.2**
    - 验证航向改变大于等于15度
    - _Requirements: 3.2_

  - [ ]* 7.7 编写直航船保持的属性测试
    - **Property 9: 直航船的航向保持**
    - **Validates: Requirements 3.3**
    - _Requirements: 3.3_

- [ ] 8. Checkpoint - 风险评估和规则引擎验证
  - 运行所有风险评估和规则引擎测试
  - 验证COLREGS规则的正确性
  - 确保所有测试通过，询问用户是否有问题


- [ ] 9. 实现路径规划模块
  - [ ] 9.1 创建路径规划模块文件
    - 在 collision_avoidance 包中创建 path_planning.py
    - 定义 Path, AvoidanceAction 等数据类
    - _Requirements: 5.1-5.6_

  - [ ] 9.2 实现候选路径生成函数
    - 实现 generate_avoidance_paths() 函数
    - 生成多种避让策略（转向、减速、组合）
    - _Requirements: 5.1_

  - [ ]* 9.3 编写路径生成的单元测试
    - 测试各种避让策略的生成
    - _Requirements: 5.1_

  - [ ] 9.4 实现路径评估函数
    - 实现 evaluate_path() 函数
    - 评估安全性、效率、合规性
    - _Requirements: 5.2, 5.3, 5.4_

  - [ ]* 9.5 编写路径安全性验证的属性测试
    - **Property 12: 路径安全性验证**
    - **Validates: Requirements 5.3**
    - 验证选择的路径不会引发新的碰撞风险
    - _Requirements: 5.3_

  - [ ]* 9.6 编写路径COLREGS合规性的属性测试
    - **Property 13: 路径COLREGS合规性**
    - **Validates: Requirements 5.4**
    - _Requirements: 5.4_

  - [ ] 9.7 实现路径选择和控制指令生成
    - 选择最优路径
    - 生成航向和航速调整指令
    - _Requirements: 5.5_

  - [ ] 9.8 实现返航路径规划
    - 实现避让完成后返回原航线的逻辑
    - _Requirements: 5.6_

  - [ ]* 9.9 编写返航路径的属性测试
    - **Property 14: 避让后的返航路径**
    - **Validates: Requirements 5.6**
    - _Requirements: 5.6_

- [ ] 10. 集成避碰决策模块到ROS2系统
  - [ ] 10.1 创建避碰决策ROS2节点
    - 创建 collision_avoidance_node.py
    - 定义 CollisionAvoidanceNode 类，继承自 rclpy.node.Node
    - 订阅 /ais/ship_states 话题
    - 发布 /collision_avoidance/decisions 话题
    - _Requirements: 3.1-3.6, 4.1-4.5, 5.1-5.6_

  - [ ] 10.2 实现避碰决策主循环
    - 在节点回调中集成风险评估、规则引擎、路径规划
    - 实现完整的避碰决策流程
    - _Requirements: 3.1-3.6, 4.1-4.5, 5.1-5.6_

  - [ ] 10.3 定义控制指令消息类型
    - 创建 ControlCommand.msg 消息定义
    - 包含航向调整、速度调整等字段
    - _Requirements: 5.5_

  - [ ] 10.4 修改AIS模拟器以接收控制指令
    - 在 ais_sim_node.py 中添加控制指令订阅器
    - 应用控制指令到船舶状态更新逻辑
    - 替换现有的简单避碰逻辑
    - _Requirements: 5.5_

  - [ ]* 10.5 编写集成测试
    - 测试完整的避碰流程
    - 从场景生成到避让完成
    - _Requirements: 1.1-1.5, 3.1-3.6, 4.1-4.5, 5.1-5.6_

- [ ] 11. Checkpoint - 避碰决策模块验证
  - 运行集成测试
  - 手动运行几个场景验证避碰效果
  - 确保所有测试通过，询问用户是否有问题

- [ ] 12. 实现测试评估框架
  - [ ] 12.1 创建性能指标数据模型
    - 在 test_framework 中创建 metrics.py
    - 实现 PerformanceMetrics 数据类
    - 定义所有评估指标
    - _Requirements: 6.1-6.5_

  - [ ] 12.2 实现场景运行和数据收集
    - 创建 test_runner.py
    - 实现 TestRunner 类
    - 运行场景并收集数据
    - _Requirements: 6.1-6.6_

  - [ ] 12.3 实现性能指标计算
    - 计算避碰成功率、平均距离等指标
    - _Requirements: 6.1-6.5_

  - [ ]* 12.4 编写性能指标计算的单元测试
    - 测试各项指标的计算逻辑
    - _Requirements: 6.1-6.5_

  - [ ] 12.5 实现批量测试和报告生成
    - 运行多个场景
    - 生成性能对比报告
    - _Requirements: 6.6_

  - [ ]* 12.6 编写报告生成的单元测试
    - 测试报告文件生成
    - _Requirements: 6.6_

- [ ] 13. 实现数据记录模块
  - [ ] 13.1 创建数据记录ROS2节点
    - 创建 data_logger_node.py
    - 创建 DataLoggerNode 类
    - 订阅所有关键话题
    - _Requirements: 9.1-9.3_

  - [ ] 13.2 实现AIS数据记录
    - 记录所有船舶的AIS数据
    - 写入时序数据库或文件
    - _Requirements: 9.1_

  - [ ]* 13.3 编写AIS数据记录的属性测试
    - **Property 15: 数据记录的完整性**
    - **Validates: Requirements 9.1**
    - 验证记录的数据点数正确
    - _Requirements: 9.1_

  - [ ] 13.4 实现避让决策记录
    - 记录所有避让决策和控制指令
    - _Requirements: 9.2_

  - [ ]* 13.5 编写决策记录的属性测试
    - **Property 16: 决策记录的时间戳一致性**
    - **Validates: Requirements 9.2**
    - _Requirements: 9.2_

  - [ ] 13.6 实现中间计算结果记录
    - 记录DCPA、TCPA、CRI等中间结果
    - _Requirements: 9.3_

  - [ ] 13.7 实现数据导出功能
    - 支持CSV、JSON、ROS Bag格式导出
    - _Requirements: 9.4_

  - [ ]* 13.8 编写数据导出的round-trip测试
    - **Property 17: 数据导出格式的可解析性**
    - **Validates: Requirements 9.4**
    - 验证导出后能正确解析
    - _Requirements: 9.4_

- [ ] 14. Checkpoint - 数据记录和评估验证
  - 运行完整的场景，验证数据记录
  - 检查导出的数据文件
  - 验证性能报告生成
  - 确保所有测试通过，询问用户是否有问题

- [ ] 15. 可选功能扩展（MVP后）
  - [ ]* 15.1 实现多船包围场景生成器
    - 生成目标船被多艘船舶包围的场景
    - _Requirements: 2.2_

  - [ ]* 15.2 实现受限水域场景生成器
    - 添加狭窄航道和港口水域约束
    - 集成地图边界
    - _Requirements: 2.3_

  - [ ]* 15.3 实现恶劣天气场景生成器
    - 在船舶运动模型中添加风浪干扰
    - _Requirements: 2.4_

  - [ ]* 15.4 实现能见度不良场景生成器
    - 调整碰撞风险阈值
    - 修改避让时机参数
    - _Requirements: 2.5_

  - [ ]* 15.5 实现可视化模块（基础版）
    - 创建实时参数发布节点
    - 发布DCPA、TCPA、CRI等关键参数
    - _Requirements: 7.4_

  - [ ]* 15.6 实现回放数据准备
    - 将记录的数据转换为回放格式
    - _Requirements: 7.5_

  - [ ]* 15.7 实现配置热加载功能
    - 监听配置文件变化
    - 自动重新加载参数
    - _Requirements: 8.1_

  - [ ]* 15.8 实现算法插件接口
    - 定义避碰算法的抽象接口
    - 支持动态加载自定义算法
    - _Requirements: 8.3_

- [ ] 16. Final Checkpoint - 系统集成测试
  - 运行完整的端到端测试
  - 验证所有模块协同工作
  - 运行批量场景测试
  - 生成最终性能报告
  - 确保所有测试通过，询问用户是否有问题

- [ ] 17. 文档和示例
  - [ ] 17.1 编写用户手册
    - 系统安装指南
    - 场景配置教程
    - API文档
    - _Requirements: 8.2_

  - [ ] 17.2 创建示例场景库
    - 提供10个典型场景配置
    - 包含各种相遇类型
    - _Requirements: 8.2_

  - [ ] 17.3 编写开发者文档
    - 架构说明
    - 扩展指南
    - 贡献指南
    - _Requirements: 8.3_

## Notes

- 标记为 `*` 的任务为可选任务，可以在MVP阶段跳过以加快开发速度
- 每个任务都引用了具体的需求编号，确保可追溯性
- Checkpoint任务用于阶段性验证，确保增量开发的质量
- 属性测试任务明确标注了对应的设计属性编号
- 建议按照任务顺序执行，因为后续任务依赖前面任务的成果
- 每个主要模块完成后都有Checkpoint，便于及时发现和解决问题
- 当前已完成基础设施和对遇场景生成器，下一步应实现其他场景生成器或开始碰撞风险评估模块

## 实现优先级建议

**MVP核心功能（必须完成）：**
1. 完成所有场景生成器（任务3）
2. 实现碰撞风险评估模块（任务6）
3. 实现COLREGS规则引擎（任务7）
4. 实现路径规划模块（任务9）
5. 集成到ROS2系统（任务10）

**增强功能（MVP后）：**
- 测试评估框架（任务12）
- 数据记录模块（任务13）
- 可选功能扩展（任务15）

**文档和完善（最后）：**
- 文档和示例（任务17）
