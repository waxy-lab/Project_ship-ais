#!/usr/bin/env python3
"""
避碰决策 ROS2 节点

Requirements: 3.1-3.6, 4.1-4.5, 5.1-5.6
"""
import json
import time
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from ais_msgs.msg import ControlCommand

from ais_msgs.msg import AisShipList  # AisShip unused

from collision_avoidance.risk_assessment import (
    assess_collision_risk,
    RISK_THRESHOLD_WARNING,
)
from collision_avoidance.rules_engine import (
    apply_colregs_rule,
    analyze_encounter_situation,
    ActionType,
)
from collision_avoidance.path_planning import (
    plan_avoidance,
    plan_return_path,
    ReturnPathConfig,
)
from scenario_generator.models import ShipState, EnvironmentConfig


def _ais_msg_to_ship_state(msg):
    try:
        return ShipState(
            mmsi=int(msg.mmsi),
            latitude=float(msg.latitude),
            longitude=float(msg.longitude),
            heading=float(msg.heading) % 360.0,
            sog=float(msg.sog),
            rot=float(msg.rot),
        )
    except (ValueError, AttributeError):
        return None


def _strategy_to_dict(strategy):
    if strategy is None:
        return {}
    path = strategy.path
    score = strategy.score
    commands = [
        {
            "timestamp": cmd.timestamp,
            "target_heading": cmd.target_heading,
            "target_speed": cmd.target_speed,
            "turn_rate": cmd.turn_rate,
            "duration": cmd.duration,
            "command_type": cmd.command_type,
        }
        for cmd in strategy.control_commands
    ]
    return {
        "strategy_id": strategy.strategy_id,
        "path_id": path.path_id,
        "path_type": path.path_type.value,
        "course_change": path.course_change,
        "speed_factor": path.speed_factor,
        "estimated_duration_min": path.estimated_duration,
        "score": {
            "total": round(score.total, 4),
            "safety": round(score.safety, 4),
            "efficiency": round(score.efficiency, 4),
            "compliance": round(score.compliance, 4),
        },
        "control_commands": commands,
        "reason": strategy.reason,
    }


def _build_no_risk_payload(own, reason, now=0.0):
    return {"decision_type": "no_action", "own_ship_mmsi": own.mmsi,
            "status": reason, "timestamp": now}


class CollisionAvoidanceNode(Node):
    def __init__(self):
        super().__init__("collision_avoidance_node")
        self.declare_parameter("own_ship_mmsi", 0)
        self.declare_parameter("risk_threshold", RISK_THRESHOLD_WARNING)
        self.declare_parameter("update_rate", 1.0)
        self.declare_parameter("min_publish_interval", 0.5)  # 最小发布间隔（秒）
        self._own_ship_mmsi = self.get_parameter("own_ship_mmsi").value
        self._risk_threshold = self.get_parameter("risk_threshold").value
        update_rate = self.get_parameter("update_rate").value
        self._min_publish_interval = self.get_parameter("min_publish_interval").value
        # 船舶状态
        self._ship_list = []
        self._own_ship = None
        self._environment = EnvironmentConfig()
        # 避让状态机
        self._original_heading = None
        self._original_speed = None
        self._is_avoiding = False
        self._avoidance_start_time = None
        self._last_strategy_id = None
        # 节流控制
        self._last_publish_time = 0.0
        self._decision_count = 0
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self._sub_ais = self.create_subscription(
            AisShipList, "/ais/ship_states", self._ais_callback, qos)
        self._pub_decisions = self.create_publisher(
            String, "/collision_avoidance/decisions", 10)
        self._pub_control = self.create_publisher(
            ControlCommand, "/collision_avoidance/control_commands", 10)
        self._timer = self.create_timer(update_rate, self._decision_loop)
        mmsi_label = str(self._own_ship_mmsi) if self._own_ship_mmsi else "auto"
        self.get_logger().info(
            f"避碰决策节点已启动 | own_mmsi={mmsi_label} | "
            f"risk_threshold={self._risk_threshold:.2f} | update_rate={update_rate:.1f}s")

    def _ais_callback(self, msg):
        ships = []
        for ais_ship in msg.ships:
            state = _ais_msg_to_ship_state(ais_ship)
            if state is not None:
                ships.append(state)
        if not ships:
            self.get_logger().warning("收到空的 AIS 列表，跳过本次更新。")
            return
        self._ship_list = ships
        if self._own_ship_mmsi == 0:
            self._own_ship = ships[0]
        else:
            matched = [s for s in ships if s.mmsi == self._own_ship_mmsi]
            self._own_ship = matched[0] if matched else None
            if self._own_ship is None:
                self.get_logger().warning(f"未找到 MMSI={self._own_ship_mmsi} 的本船。")
        if self._own_ship is not None and self._original_heading is None:
            self._original_heading = self._own_ship.heading
            self._original_speed = self._own_ship.sog
            self.get_logger().info(
                f"本船已识别: MMSI={self._own_ship.mmsi} | "
                f"heading={self._original_heading:.1f}deg | sog={self._original_speed:.1f}kn")

    def _decision_loop(self):
        if self._own_ship is None:
            return
        own = self._own_ship
        targets = [s for s in self._ship_list if s.mmsi != own.mmsi]
        now = time.time()

        # Requirements 3.1: 无目标船时发布安全状态
        if not targets:
            self._publish_decision(_build_no_risk_payload(own, "no_targets", now), now)
            return

        # Requirements 3.2: 对所有目标船进行风险评估
        risk_results = []
        for target in targets:
            try:
                risk = assess_collision_risk(own, target)
                risk_results.append((target, risk))
            except Exception as exc:
                self.get_logger().warning(f"风险评估异常 (MMSI={target.mmsi}): {exc}")

        # Requirements 3.3: 筛选高风险目标
        high_risk = [(t, r) for t, r in risk_results if r.cri >= self._risk_threshold]

        if not high_risk:
            # 无高风险目标
            if self._is_avoiding:
                # Requirements 5.6: 尝试返回原航线
                self._try_return(own, targets, now)
            else:
                self._publish_decision(_build_no_risk_payload(own, "safe", now), now)
            return

        # Requirements 3.4: 按 CRI 降序排列，选主要威胁
        high_risk.sort(key=lambda x: x[1].cri, reverse=True)
        primary_target, primary_risk = high_risk[0]

        # 进入避让状态
        if not self._is_avoiding:
            self._is_avoiding = True
            self._avoidance_start_time = now
            self.get_logger().info(
                f"[状态] 进入避让模式 | 主要威胁 MMSI={primary_target.mmsi} "
                f"CRI={primary_risk.cri:.3f}")

        # Requirements 4.1: 态势分析
        try:
            situation = analyze_encounter_situation(own, primary_target)
        except Exception as exc:
            self.get_logger().error(f"态势分析异常: {exc}")
            return

        # Requirements 4.2-4.5: 应用 COLREGS 规则
        try:
            colregs_action = apply_colregs_rule(situation.encounter_type, own, primary_target)
        except Exception as exc:
            self.get_logger().error(f"COLREGS规则应用异常: {exc}")
            return

        # Requirements 5.1-5.5: 路径规划（仅让路船需要）
        strategy = None
        if colregs_action.action_type not in (ActionType.MAINTAIN, ActionType.NO_ACTION):
            # 将所有高风险目标都纳入规划（综合避让）
            all_threat_ships = [t for t, _ in high_risk]
            try:
                strategy = plan_avoidance(own, all_threat_ships, self._environment, colregs_action)
                # 记录策略 ID 用于去重
                if strategy is not None:
                    self._last_strategy_id = strategy.strategy_id
            except Exception as exc:
                self.get_logger().warning(f"路径规划异常: {exc}")

        payload = self._build_decision_payload(
            own, primary_target, primary_risk, situation,
            colregs_action, strategy, risk_results, now)
        self._publish_decision(payload, now)
        self._decision_count += 1

        # Requirements 5.5: 发布控制指令到 /collision_avoidance/control_commands
        if strategy is not None and strategy.control_commands:
            self._publish_control_command(strategy, own, now)

        self.get_logger().info(
            f"[决策#{self._decision_count}] target={primary_target.mmsi} "
            f"| CRI={primary_risk.cri:.3f} "
            f"| encounter={situation.encounter_type.value} "
            f"| action={colregs_action.action_type.value} "
            f"| threats={len(high_risk)}")

    def _publish_control_command(self, strategy, own, now):
        """将路径规划结果转换为 ControlCommand 消息并发布

        Requirements: 5.5
        """
        cmd = strategy.control_commands[0]  # 取第一条（立即执行）指令
        msg = ControlCommand()
        msg.own_ship_mmsi = own.mmsi
        msg.timestamp = now
        msg.target_heading = float(cmd.target_heading) if cmd.target_heading is not None else float('nan')
        msg.target_speed = float(cmd.target_speed) if cmd.target_speed is not None else float('nan')
        msg.turn_rate = float(cmd.turn_rate) if cmd.turn_rate is not None else float('nan')
        msg.duration = float(cmd.duration) if cmd.duration is not None else 0.0
        msg.command_type = cmd.command_type or "maintain"
        msg.reason = strategy.reason or ""
        try:
            self._pub_control.publish(msg)
        except Exception as exc:
            self.get_logger().error(f"发布控制指令失败: {exc}")

    def _try_return(self, own, targets, now):
        if self._original_heading is None or self._original_speed is None:
            self._is_avoiding = False
            return
        return_config = ReturnPathConfig(
            original_heading=self._original_heading,
            original_speed=self._original_speed,
        )
        try:
            return_path = plan_return_path(own, return_config, targets)
        except Exception as exc:
            self.get_logger().warning(f"返航规划异常: {exc}")
            return_path = None
        if return_path is not None:
            self._is_avoiding = False
            self._last_strategy_id = None
            avoidance_duration = now - self._avoidance_start_time if self._avoidance_start_time else 0
            payload = {
                "decision_type": "return_to_original_course",
                "own_ship_mmsi": own.mmsi,
                "return_heading": self._original_heading,
                "return_speed": self._original_speed,
                "avoidance_duration_sec": round(avoidance_duration, 1),
                "status": "returning",
                "timestamp": now,
            }
            self.get_logger().info(
                f"[状态] 威胁解除，规划返航: heading={self._original_heading:.1f}deg "
                f"| 避让历时={avoidance_duration:.0f}s")
        else:
            payload = {
                "decision_type": "hold_avoidance",
                "own_ship_mmsi": own.mmsi,
                "status": "waiting_to_return",
                "timestamp": now,
            }
        self._publish_decision(payload, now)

    def _publish_decision(self, payload, now=None):
        # 节流控制：避免发布过于频繁
        if now is None:
            now = time.time()
        if now - self._last_publish_time < self._min_publish_interval:
            return
        self._last_publish_time = now
        try:
            msg = String()
            msg.data = json.dumps(payload, ensure_ascii=False)
            self._pub_decisions.publish(msg)
        except Exception as exc:
            self.get_logger().error(f"发布决策消息失败: {exc}")

    def _build_decision_payload(self, own, primary_target, primary_risk,
                                situation, colregs_action, strategy, all_risks, now):
        risk_summary = [
            {"target_mmsi": t.mmsi, "cri": round(r.cri, 4),
             "risk_level": r.risk_level.value,
             "dcpa_nm": round(r.dcpa, 4), "tcpa_min": round(r.tcpa, 2),
             "distance_nm": round(r.distance, 4)}
            for t, r in all_risks
        ]
        avoidance_duration = (
            round(now - self._avoidance_start_time, 1)
            if self._avoidance_start_time else 0.0
        )
        return {
            "decision_type": "avoidance",
            "own_ship_mmsi": own.mmsi,
            "timestamp": now,
            "avoidance_duration_sec": avoidance_duration,
            "primary_threat": {
                "mmsi": primary_target.mmsi,
                "cri": round(primary_risk.cri, 4),
                "risk_level": primary_risk.risk_level.value,
                "dcpa_nm": round(primary_risk.dcpa, 4),
                "tcpa_min": round(primary_risk.tcpa, 2),
                "distance_nm": round(primary_risk.distance, 4),
                "relative_bearing_deg": round(primary_risk.relative_bearing, 2),
            },
            "encounter_type": situation.encounter_type.value,
            "own_ship_role": situation.own_ship_role.value,
            "colregs_action": {
                "action_type": colregs_action.action_type.value,
                "turn_direction": (colregs_action.turn_direction.value
                                   if colregs_action.turn_direction else None),
                "turn_angle_deg": colregs_action.turn_angle,
                "speed_factor": colregs_action.speed_factor,
                "reason": colregs_action.reason,
            },
            "avoidance_strategy": _strategy_to_dict(strategy),
            "all_risks": risk_summary,
        }


def main(args=None):
    rclpy.init(args=args)
    node = CollisionAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("收到 Ctrl+C，正在关闭...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
