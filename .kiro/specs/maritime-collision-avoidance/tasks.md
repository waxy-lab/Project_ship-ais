# Implementation Plan: 水路仿真测试框架与智能避碰系统

## Overview

本实现计划将系统开发分为4个主要阶段，每个阶段包含多个可执行的编码任务。任务按照依赖关系排序，确保每一步都能在前一步的基础上进行。标记为`*`的任务为可选任务，可以在MVP阶段跳过。

## Tasks

- [x] 1. 项目结构重组和基础设施搭建
  - 重组现有代码结构，创建新的模块目录
  - 设置测试框架（pytest + Hypothesis）
  - 创建配置管理系统
  - _Requirements: 8.1, 8.2, 10.1_

- [x] 1.1 编写项目结构重组脚本
  - 创建新的包结构：scenario_generator, collision_avoidance, test_framework
  - 迁移现有代码到新结构    
  - _Requirements: 8.1_

- [x] 1.2 配置pytest和Hypothesis测试框架
  - 安装依赖：pytest, hypothesis, pytest-ros
  - 创建测试配置文件 pytest.ini
  - 编写测试工具函数和策略生成器
  - _Requirements: 6.1-6.6_

- [ ] 2. 实现场景生成器核心功能
  - [x] 2.1 创建场景配置数据模型
    - 实现 ScenarioConfig, ShipState, EnvironmentConfig 数据类
    - 添加数据验证逻辑
    - _Requirements: 1.1-1.4, 8.2_

  - [ ] 2.2 编写场景配置数据模型的单元测试
    - 测试数据验证逻辑
    - 测试边界情况（无效坐标、负数速度等）
    - _Requirements: 1.1-1.4_

  - [ ] 2.3 实现对遇场景生成器
    - 实现 generate_head_on_scenario() 方法
    - 计算两船初始位置和航向
    - _Requirements: 1.1_

  - [ ]* 2.4 编写对遇场景生成的属性测试
    - **Property 1: 场景生成的几何正确性**
    - **Validates: Requirements 1.1**
    - 使用Hypothesis生成随机参数，验证几何条件
    - _Requirements: 1.1_

  - [ ] 2.5 实现交叉相遇场景生成器
    - 实现 generate_crossing_scenario() 方法
    - 计算交叉角度和初始位置
    - _Requirements: 1.2_

  - [ ]* 2.6 编写交叉场景生成的属性测试
    - **Property 2: 交叉场景的角度约束**
    - **Validates: Requirements 1.2**
    - _Requirements: 1.2_

  - [ ] 2.7 实现追越场景生成器
    - 实现 generate_overtaking_scenario() 方法
    - 确保追越船在后方扇形区域且速度更快
    - _Requirements: 1.3_

  - [ ]* 2.8 编写追越场景生成的属性测试
    - **Property 3: 追越场景的速度和位置关系**
    - **Validates: Requirements 1.3**
    - _Requirements: 1.3_

  - [ ] 2.9 实现多船复杂场景生成器
    - 实现 generate_multi_ship_scenario() 方法
    - 支持3艘以上船舶的场景
    - _Requirements: 1.4_

  - [ ]* 2.10 编写多船场景生成的单元测试
    - 测试船舶数量和初始状态
    - _Requirements: 1.4_

- [ ] 3. Checkpoint - 场景生成器验证
  - 运行所有场景生成器测试
  - 手动验证生成的场景配置文件
  - 确保所有测试通过，询问用户是否有问题

- [ ] 4. 实现碰撞风险评估模块
  - [ ] 4.1 实现DCPA/TCPA计算函数
    - 实现 calculate_dcpa_tcpa() 函数
    - 使用相对运动矢量法
    - _Requirements: 4.1_

  - [ ]* 4.2 编写DCPA/TCPA计算的属性测试
    - **Property 6: DCPA/TCPA计算的对称性**
    - **Validates: Requirements 4.1**
    - 验证从两个视角计算结果一致
    - _Requirements: 4.1_

  - [ ]* 4.3 编写DCPA/TCPA计算的单元测试
    - 测试平行航行、相向航行等特殊情况
    - 测试边界情况（静止船舶、相同位置等）
    - _Requirements: 4.1_

  - [ ] 4.4 实现CRI（碰撞风险指数）计算函数
    - 实现 calculate_cri() 函数
    - 综合考虑距离、时间、方位、速度比等因素
    - _Requirements: 4.2_

  - [ ]* 4.5 编写CRI计算的属性测试
    - **Property 10: CRI值的单调性**
    - **Validates: Requirements 4.2**
    - 验证DCPA/TCPA减小时CRI增加
    - _Requirements: 4.2_

  - [ ] 4.6 实现风险阈值判定逻辑
    - 实现预警阈值和危险阈值判定
    - 发出相应的信号
    - _Requirements: 4.3, 4.4_

  - [ ]* 4.7 编写风险阈值触发的属性测试
    - **Property 11: 风险阈值触发的一致性**
    - **Validates: Requirements 4.3, 4.4**
    - _Requirements: 4.3, 4.4_

- [ ] 5. 实现COLREGS规则引擎
  - [ ] 5.1 实现相遇类型判定函数
    - 实现 determine_encounter_type() 函数
    - 判断对遇、交叉、追越等类型
    - _Requirements: 3.1-3.4_

  - [ ]* 5.2 编写相遇类型判定的单元测试
    - 测试各种相遇类型的判定
    - 测试边界情况
    - _Requirements: 3.1-3.4_

  - [ ] 5.3 实现COLREGS规则应用函数
    - 实现 apply_colregs_rule() 函数
    - 根据相遇类型返回避让动作
    - _Requirements: 3.1-3.6_

  - [ ]* 5.4 编写对遇规则的属性测试
    - **Property 7: 对遇规则的转向方向**
    - **Validates: Requirements 3.1**
    - 验证对遇时向右转向
    - _Requirements: 3.1_

  - [ ]* 5.5 编写让路船避让的属性测试
    - **Property 8: 让路船的明显避让**
    - **Validates: Requirements 3.2**
    - 验证航向改变大于等于15度
    - _Requirements: 3.2_

  - [ ]* 5.6 编写直航船保持的属性测试
    - **Property 9: 直航船的航向保持**
    - **Validates: Requirements 3.3**
    - _Requirements: 3.3_

- [ ] 6. Checkpoint - 风险评估和规则引擎验证
  - 运行所有风险评估和规则引擎测试
  - 验证COLREGS规则的正确性
  - 确保所有测试通过，询问用户是否有问题


- [ ] 7. 实现路径规划模块
  - [ ] 7.1 实现候选路径生成函数
    - 实现 generate_avoidance_paths() 函数
    - 生成多种避让策略（转向、减速、组合）
    - _Requirements: 5.1_

  - [ ]* 7.2 编写路径生成的单元测试
    - 测试各种避让策略的生成
    - _Requirements: 5.1_

  - [ ] 7.3 实现路径评估函数
    - 实现 evaluate_path() 函数
    - 评估安全性、效率、合规性
    - _Requirements: 5.2, 5.3, 5.4_

  - [ ]* 7.4 编写路径安全性验证的属性测试
    - **Property 12: 路径安全性验证**
    - **Validates: Requirements 5.3**
    - 验证选择的路径不会引发新的碰撞风险
    - _Requirements: 5.3_

  - [ ]* 7.5 编写路径COLREGS合规性的属性测试
    - **Property 13: 路径COLREGS合规性**
    - **Validates: Requirements 5.4**
    - _Requirements: 5.4_

  - [ ] 7.6 实现路径选择和控制指令生成
    - 选择最优路径
    - 生成航向和航速调整指令
    - _Requirements: 5.5_

  - [ ] 7.7 实现返航路径规划
    - 实现避让完成后返回原航线的逻辑
    - _Requirements: 5.6_

  - [ ]* 7.8 编写返航路径的属性测试
    - **Property 14: 避让后的返航路径**
    - **Validates: Requirements 5.6**
    - _Requirements: 5.6_

- [ ] 8. 集成避碰决策模块到现有AIS模拟器
  - [ ] 8.1 创建避碰决策ROS2节点
    - 创建 CollisionAvoidanceNode 类
    - 订阅 /ais/ship_states 话题
    - 发布 /control/commands 话题
    - _Requirements: 3.1-3.6, 4.1-4.5, 5.1-5.6_

  - [ ] 8.2 修改AIS模拟器以接收控制指令
    - 修改 ais_sim_node.py
    - 添加控制指令订阅器
    - 应用控制指令到船舶状态
    - _Requirements: 5.5_

  - [ ]* 8.3 编写集成测试
    - 测试完整的避碰流程
    - 从场景生成到避让完成
    - _Requirements: 1.1-1.5, 3.1-3.6, 4.1-4.5, 5.1-5.6_

- [ ] 9. Checkpoint - 避碰决策模块验证
  - 运行集成测试
  - 手动运行几个场景验证避碰效果
  - 确保所有测试通过，询问用户是否有问题

- [ ] 10. 实现极端场景生成器
  - [ ] 10.1 实现紧急避让场景生成器
    - 实现 generate_emergency_scenario() 方法
    - 生成DCPA<0.5海里且TCPA<5分钟的场景
    - _Requirements: 2.1_

  - [ ]* 10.2 编写紧急场景的属性测试
    - **Property 5: 危险场景的风险参数**
    - **Validates: Requirements 2.1**
    - _Requirements: 2.1_

  - [ ] 10.3 实现多船包围场景生成器
    - 生成目标船被多艘船舶包围的场景
    - _Requirements: 2.2_

  - [ ] 10.4 实现受限水域场景生成器
    - 添加狭窄航道和港口水域约束
    - 集成地图边界
    - _Requirements: 2.3_

  - [ ] 10.5 实现恶劣天气场景生成器
    - 在船舶运动模型中添加风浪干扰
    - _Requirements: 2.4_

  - [ ] 10.6 实现能见度不良场景生成器
    - 调整碰撞风险阈值
    - 修改避让时机参数
    - _Requirements: 2.5_

- [ ] 11. 实现测试评估框架
  - [ ] 11.1 创建性能指标数据模型
    - 实现 PerformanceMetrics 数据类
    - 定义所有评估指标
    - _Requirements: 6.1-6.5_

  - [ ] 11.2 实现场景运行和数据收集
    - 创建 TestRunner 类
    - 运行场景并收集数据
    - _Requirements: 6.1-6.6_

  - [ ] 11.3 实现性能指标计算
    - 计算避碰成功率、平均距离等指标
    - _Requirements: 6.1-6.5_

  - [ ]* 11.4 编写性能指标计算的单元测试
    - 测试各项指标的计算逻辑
    - _Requirements: 6.1-6.5_

  - [ ] 11.5 实现批量测试和报告生成
    - 运行多个场景
    - 生成性能对比报告
    - _Requirements: 6.6_

  - [ ]* 11.6 编写报告生成的单元测试
    - 测试报告文件生成
    - _Requirements: 6.6_

- [ ] 12. 实现数据记录模块
  - [ ] 12.1 创建数据记录ROS2节点
    - 创建 DataLoggerNode 类
    - 订阅所有关键话题
    - _Requirements: 9.1-9.3_

  - [ ] 12.2 实现AIS数据记录
    - 记录所有船舶的AIS数据
    - 写入时序数据库或文件
    - _Requirements: 9.1_

  - [ ]* 12.3 编写AIS数据记录的属性测试
    - **Property 15: 数据记录的完整性**
    - **Validates: Requirements 9.1**
    - 验证记录的数据点数正确
    - _Requirements: 9.1_

  - [ ] 12.4 实现避让决策记录
    - 记录所有避让决策和控制指令
    - _Requirements: 9.2_

  - [ ]* 12.5 编写决策记录的属性测试
    - **Property 16: 决策记录的时间戳一致性**
    - **Validates: Requirements 9.2**
    - _Requirements: 9.2_

  - [ ] 12.6 实现中间计算结果记录
    - 记录DCPA、TCPA、CRI等中间结果
    - _Requirements: 9.3_

  - [ ] 12.7 实现数据导出功能
    - 支持CSV、JSON、ROS Bag格式导出
    - _Requirements: 9.4_

  - [ ]* 12.8 编写数据导出的round-trip测试
    - **Property 17: 数据导出格式的可解析性**
    - **Validates: Requirements 9.4**
    - 验证导出后能正确解析
    - _Requirements: 9.4_

- [ ] 13. Checkpoint - 数据记录和评估验证
  - 运行完整的场景，验证数据记录
  - 检查导出的数据文件
  - 验证性能报告生成
  - 确保所有测试通过，询问用户是否有问题

- [ ] 14. 实现可视化模块（基础版）
  - [ ] 14.1 创建实时参数发布节点
    - 发布DCPA、TCPA、CRI等关键参数
    - 供外部可视化工具订阅
    - _Requirements: 7.4_

  - [ ]* 14.2 编写参数发布的属性测试
    - **Property 18: 实时参数的正确计算**
    - **Validates: Requirements 7.4**
    - _Requirements: 7.4_

  - [ ] 14.3 实现回放数据准备
    - 将记录的数据转换为回放格式
    - _Requirements: 7.5_

  - [ ]* 14.4 编写回放功能的属性测试
    - **Property 19: 回放数据的完整性**
    - **Validates: Requirements 7.5**
    - _Requirements: 7.5_

- [ ] 15. 实现配置管理和扩展性
  - [ ] 15.1 创建统一的配置管理系统
    - 支持YAML配置文件
    - 支持参数验证
    - _Requirements: 8.1, 8.2_

  - [ ] 15.2 实现配置热加载功能
    - 监听配置文件变化
    - 自动重新加载参数
    - _Requirements: 8.1_

  - [ ]* 15.3 编写配置热加载的测试
    - **Property 20: 配置文件的热加载**
    - **Validates: Requirements 8.1**
    - _Requirements: 8.1_

  - [ ] 15.4 实现算法插件接口
    - 定义避碰算法的抽象接口
    - 支持动态加载自定义算法
    - _Requirements: 8.3_

  - [ ] 15.5 实现标准AIS数据格式解析
    - 支持NMEA 0183格式
    - 支持ROS2消息格式
    - _Requirements: 8.4_

  - [ ]* 15.6 编写AIS格式解析的属性测试
    - **Property 21: 标准AIS格式的解析**
    - **Validates: Requirements 8.4**
    - _Requirements: 8.4_

- [ ] 16. 实现系统部署和运维功能
  - [ ] 16.1 创建一键安装脚本
    - 自动安装所有依赖
    - 配置ROS2环境
    - _Requirements: 10.1_

  - [ ] 16.2 实现系统启动检查
    - 检查依赖项完整性
    - 检查配置文件有效性
    - _Requirements: 10.2_

  - [ ]* 16.3 编写启动检查的属性测试
    - **Property 22: 系统启动的依赖检查**
    - **Validates: Requirements 10.2**
    - _Requirements: 10.2_

  - [ ] 16.4 实现健康检查接口
    - 提供ROS2服务接口
    - 返回系统健康状态
    - _Requirements: 10.3_

  - [ ] 16.5 实现错误处理和恢复机制
    - 详细的错误信息
    - 自动恢复建议
    - _Requirements: 10.4_

  - [ ]* 16.6 创建Docker镜像和编排配置
    - 编写Dockerfile
    - 创建docker-compose.yml
    - _Requirements: 10.5_

- [ ] 17. Final Checkpoint - 系统集成测试
  - 运行完整的端到端测试
  - 验证所有模块协同工作
  - 运行批量场景测试
  - 生成最终性能报告
  - 确保所有测试通过，询问用户是否有问题

- [ ] 18. 文档和示例
  - [ ] 18.1 编写用户手册
    - 系统安装指南
    - 场景配置教程
    - API文档
    - _Requirements: 8.2_

  - [ ] 18.2 创建示例场景库
    - 提供10个典型场景配置
    - 包含各种相遇类型
    - _Requirements: 8.2_

  - [ ] 18.3 编写开发者文档
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
