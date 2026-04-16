# 系统整体融合任务清单
## 目标
构建一个完整的“航线感知场景生成 + 规则驱动本船避碰 + 模拟执行”闭环系统：
1. 以真实主航线作为场景锚点，而不是仅以单个基准点生成标准场景。
2. 由场景生成模块在主航线不同区段自动生成对遇、交叉、追越等目标船场景。
3. 由 AIS 模拟器执行本船主航线与目标船预设轨迹。
4. 由避碰决策模块仅对本船进行 COLREGS 风险评估、规则判定和路径规划。
5. 支持生成可复现、可配置、可测试、可演示的多场景综合仿真。
- 明确职责边界：
  - `scenario_generator` 负责“生成什么场景、把船放在哪里、何时进场”。
  - `collision_avoidance` 负责“本船如何按规则避碰”。
  - `ais_simulator` 负责“如何执行船舶运动与控制命令”。
- 目标船默认按预设轨迹航行；本船默认启用规则避碰。
- 所有新增格式都需要兼容现有 `simulated_ships` 运行格式。
---
## Phase 1：统一场景数据模型
- [x] 1. 扩展 `scenario_generator` 数据模型，支持航线感知场景
  - 在 `src/scenario_generator/scenario_generator/models.py` 中新增或扩展：
    - `RouteScenarioConfig`
    - `EncounterSpec`
    - `RouteRef` / `RouteSegmentRef`
  - 支持以下概念：
    - 本船主航线 `own_ship_route`
    - 场景锚点 `route_progress` 或 `route_segment_index`
    - 场景类型 `head_on / crossing / overtaking`
    - 目标船启动/停止时间 `start_time / end_time`
    - 船舶角色 `own_ship / target_ship`
    - 是否启用避碰 `collision_avoidance_enabled`
  - 保持与现有 `ScenarioConfig` 并存，不破坏已有单场景生成能力。
- [x] 2. 为 `ShipState` 增加系统级运行字段
  - 在 `ShipState` 中补充可选字段：
    - `start_time`
    - `end_time`
    - `role`
    - `encounter_type`
    - `collision_avoidance_enabled`
    - `route_id`
  - 更新 `to_dict()` / `from_dict()`，确保能双向转换。
- [ ] 3. 定义新的航线场景 YAML 规范
  - 在 `scenario_generator/config/scenarios/` 下新增示例规范。
  - 格式至少包含：
    - 地图/水域标识
    - 本船主航线
    - encounters 列表
    - 环境配置
    - 仿真时长
  - 输出一份规范示例，作为后续生成器与仿真器的统一输入。
---
## Phase 2：实现“基于主航线的场景编排器”
- [x] 4. 在 `scenario_generator` 中新增 route-aware orchestration 模块
  - 新增文件建议：`src/scenario_generator/scenario_generator/route_scenario_generator.py`
  - 输入：
    - 本船主航线
    - encounter 列表
    - 环境配置
  - 输出：
    - 完整 `ScenarioConfig` 或兼容对象
    - 其中包含本船 + 多艘目标船的初始状态与 waypoints
- [x] 5. 实现主航线几何工具
  - 新增公共几何能力：
    - 计算航线总长度
    - 根据 `route_progress` 取锚点
    - 计算航线局部切向航向
    - 计算法向左右偏移
    - 将“海里级偏移”映射到经纬度
  - 用于在主航线某段上布置交叉/对遇/追越船。
- [x] 6. 实现追越场景挂接逻辑
  - 在主航线前方同向放置慢船。
  - 自动生成更贴近本船航线的目标船 waypoints。
  - 支持参数：
    - 前向距离
    - 相对速度差
    - 起始时间
    - 结束时间
  - 保证目标船位于本船前方合理区间，而不是偏离成平行旁路。
- [x] 7. 实现交叉场景挂接逻辑
  - 以主航线某锚点为交叉点，生成右舷/左舷进入船。
  - 支持按目标 TCPA/DCPA 反算目标船起点与航向。
  - 支持参数：
    - `crossing_side`
    - `crossing_angle`
    - `target_tcpa`
    - `target_dcpa`
- [x] 8. 实现对遇场景挂接逻辑
  - 以主航线某后段锚点为会遇点，生成反向接近船。
  - 自动生成从前方水域进入并沿近似反向航线靠近的轨迹。
  - 支持控制对遇距离、起始时间、进入方向。
- [ ] 9. 实现多场景编排与去冲突规则
  - 支持在同一主航线上编排多个 encounter。
  - 增加编排约束：
    - 场景之间的最小时间间隔
    - 场景之间的最小空间间隔
    - 避免目标船彼此形成非预期会遇
  - 当场景冲突时，给出调整建议或自动重排。
---
## Phase 3：打通生成器到模拟器的运行链路
- [x] 10. 扩展 `AISConfigConverter`，保留系统级语义字段
  - 更新 `src/scenario_generator/scenario_generator/ais_converter.py`
  - 不仅输出：
    - `mmsi / latitude / longitude / heading / sog / rot / waypoints`
  - 还输出：
    - `start_time`
    - `end_time`
    - `role`
    - `encounter_type`
    - `collision_avoidance_enabled`
  - 保证生成结果可直接被 `ais_simulator` 使用。
- [x] 11. 扩展 `ScenarioLoader` 以支持航线场景规范
  - 更新 `src/scenario_generator/scenario_generator/scenario_loader.py`
  - 新增对 route-aware scenario YAML 的识别与分派。
  - 保留对旧 `scenario_type` 格式的兼容。
- [x] 12. 升级 `wenshan_scenario_node.py`
  - 让 `src/wenshan_pro/wenshan_pro/wenshan_scenario_node.py` 支持：
    - 加载航线场景 YAML
    - 调用 route-aware generator
    - 输出完整 AIS 运行配置
  - 新增可选 CLI 参数：
    - 指定输出路径
    - 指定地图/航线模板
    - 指定是否覆盖 install 配置
- [ ] 13. 建立“源场景 -> 运行配置”文档和脚本
  - 统一生成命令。
  - 明确哪个 YAML 是高层定义，哪个 YAML 是运行产物。
  - 补充自动化脚本，避免继续手工改运行文件。
---
## Phase 4：让模拟器支持角色化执行
- [x] 14. 在 `ais_simulator` 中支持船舶角色与控制开关
  - 更新 `src/ais_simulator/ais_simulator/ais_sim_node.py`
  - 支持读取并使用：
    - `role`
    - `collision_avoidance_enabled`
    - `start_time / end_time`
  - 明确默认行为：
    - 本船：允许被避碰控制接管
    - 目标船：默认仅按 waypoint 航行
- [x] 15. 替换现有“所有船统一简单避碰”逻辑
  - 当前 `avoid_collision_with_other_ships()` 是全体船只共用的简化逻辑。
  - 目标改为：
    - 未启用避碰的目标船不参与此逻辑
    - 启用避碰的本船由 `collision_avoidance` 控制为主
    - 简化避碰逻辑只作为可选 fallback，而非默认主逻辑
- [ ] 16. 细化船舶状态发布策略
  - 对未到 `start_time` 的船舶支持两种模式：
    - 不发布
    - 发布但标记 inactive
  - 明确对 `end_time` 到期船舶的停船和状态输出行为。
- [ ] 17. 支持按 MMSI/role 识别本船
  - 确保 `collision_avoidance_node` 与 `ais_simulator` 对“谁是本船”认知一致。
  - 减少当前依赖“列表第一个就是本船”的隐含假设。
---
## Phase 5：让避碰模块只控制本船并真正闭环
- [x] 18. 将 `collision_avoidance` 改为“仅对本船决策”
  - 在 `src/collision_avoidance/collision_avoidance/collision_avoidance_node.py` 中：
    - 强制基于 `own_ship_mmsi` 或 `role=own_ship` 识别本船
    - 所有目标船仅作为风险评估对象
  - 保证控制指令只发布给本船。
- [ ] 19. 增加 encounter 级调试与决策可视化信息
  - 输出每个目标船的：
    - encounter type
    - role 判定
    - CRI/TCPA/DCPA
    - 规划动作
  - 便于验证生成场景是否触发了预期规则。
- [ ] 20. 将路径规划模块与主航线回归逻辑打通
  - 让 `plan_return_path()` 能够参考本船原始主航线，而不只是原始航向。
  - 支持避让结束后回归主航线最近点，而不是简单回正。
- [ ] 21. 增加“演示模式 / 全自动模式”开关
  - 演示模式：
    - 目标船严格按 сценарий 航行
    - 本船按规则避碰
  - 全自动模式：
    - 可扩展为更多船参与决策
  - 先实现演示模式，后保留扩展点。
---
## Phase 6：加入地图与水域约束感知
- [ ] 22. 将地图约束纳入场景生成阶段
  - 复用 `jamaica_bay.geojson` 或统一水域边界。
  - 在场景生成阶段就过滤不合理初始点和 waypoint。
  - 避免“生成后再手工挪船”。
- [ ] 23. 实现水域合法性检查工具
  - 检查：
    - 初始点是否在水域内
    - 航点是否落在水域内
    - 连线是否穿陆地
  - 生成失败时给出可读报错，而不是运行后在地图里才发现。
- [ ] 24. 对主航线场景生成加入可通航约束
  - 对交叉船、对遇船、追越船的进入位置，优先从可通航水域采样。
  - 减少目标船贴岸、上岸、穿岸的情况。
---
## Phase 7：测试、验证与交付
- [ ] 25. 为 route-aware generator 编写单元测试
  - 测试主航线锚点选取、局部切向计算、左右偏移正确性。
  - 测试多场景布置后船位合理。
- [ ] 26. 编写属性测试，验证 encounter 几何关系
  - 追越：目标船应位于本船前方同向扇区。
  - 交叉：目标船应从指定舷侧进入。
  - 对遇：目标船应满足近似反向航向。
- [ ] 27. 编写集成测试，验证“生成 -> 仿真 -> 避碰”闭环
  - 输入一条主航线和 encounter 列表。
  - 检查：
    - 运行配置生成成功
    - 本船收到控制指令
    - 目标船不被错误控制
    - encounter 类型与预期一致
- [ ] 28. 编写回归测试，覆盖当前已有标准场景
  - 保证新增 route-aware 能力不破坏：
    - head_on_basic
    - crossing
    - overtaking
    - emergency
- [ ] 29. 补充用户文档与开发文档
  - 如何定义一条主航线
  - 如何在航线上编排多个场景
  - 如何生成 AIS 运行配置
  - 如何启动本船规则避碰闭环
---
## 优先级建议
### P0：先打通主闭环
优先完成以下任务：
- 1, 2, 4, 5, 6, 7, 8, 10, 11, 14, 15, 18
完成后即可具备：
- 在指定主航线上自动生成多类场景
- 本船按规则避碰
- 目标船按预设轨迹运行
### P1：提升系统稳定性
- 9, 16, 17, 20, 22, 23, 24
完成后系统会更适合真实地图和复杂场景。
### P2：测试与文档收口
- 25, 26, 27, 28, 29
---
## 交付标准
当以下条件满足时，视为整体功能完成：
- 可以输入一条真实主航线和多个 encounter 定义。
- 系统能够自动生成完整多船运行配置，而不是手工拼 YAML。
- 本船能够由 `collision_avoidance` 模块按 COLREGS 规则避碰。
- 目标船默认不会与本船一起做对称式避让。
- 生成结果满足地图水域约束，不再频繁出现船在岸上。
- 整个流程可通过命令行或单个入口脚本稳定运行。
