#!/usr/bin/env python3
"""
避碰决策 ROS2 节点

Requirements: 3.1-3.6, 4.1-4.5, 5.1-5.6
"""
import json
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String

from ais_simulator.msg import AisShipList, AisShip

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


def _build_no_risk_payload(own, reason):
    return {"decision_type": "no_action", "own_ship_mmsi": own.mmsi, "status": reason}


class CollisionAvoidanceNode(Node):
    def __init__(self):
        super().__init__("collision_avoidance_node")
        self.declare_parameter("own_ship_mmsi", 0)
        self.declare_parameter("risk_threshold", RISK_THRESHOLD_WARNING)
        self.declare_parameter("update_rate", 1.0)
        self._own_ship_mmsi = self.get_parameter("own_ship_mmsi").value
        self._risk_threshold = self.get_parameter("risk_threshold").value
        update_rate = self.get_parameter("update_rate").value
        self._ship_list = []
        self._own_ship = None
        self._environment = EnvironmentConfig()
        self._original_heading = None
        self._original_speed = None
        self._is_avoiding = False
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self._sub_ais = self.create_subscription(
            AisShipList, "/ais/ship_states", self._ais_callback, qos)
        self._pub_decisions = self.create_publisher(
            String, "/collision_avoidance/decisions", 10)
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
            self.get_logger().warn("收到空的 AIS 列表，跳过本次更新。")
            return
        self._ship_list = ships
        if self._own_ship_mmsi == 0:
            self._own_ship = ships[0]
        else:
            matched = [s for s in ships if s.mmsi == self._own_ship_mmsi]
            self._own_ship = matched[0] if matched else None
            if self._own_ship is None:
                self.get_logger().warn(f"未找到 MMSI={self._own_ship_mmsi} 的本船。")
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
        if not targets:
            self._publish_decision(_build_no_risk_payload(own, "no_targets"))
            return
        risk_results = []
        for target in targets:
            try:
                risk = assess_collision_risk(own, target)
                risk_results.append((target, risk))
            except Exception as exc:
                self.get_logger().warn(f"风险评估异常 (MMSI={target.mmsi}): {exc}")
        high_risk = [(t, r) for t, r in risk_results if r.cri >= self._risk_threshold]
        if not high_risk:
            if self._is_avoiding:
                self._try_return(own, targets)
            else:
                self._publish_decision(_build_no_risk_payload(own, "safe"))
            return
        high_risk.sort(key=lambda x: x[1].cri, reverse=True)
        primary_target, primary_risk = high_risk[0]
        self._is_avoiding = True
        try:
            situation = analyze_encounter_situation(own, primary_target)
        except Exception as exc:
            self.get_logger().error(f"态势分析异常: {exc}")
            return
        try:
            colregs_action = apply_colregs_rule(situation.encounter_type, own, primary_target)
        except Exception as exc:
            self.get_logger().error(f"COLREGS规则应用异常: {exc}")
            return
        strategy = None
        if colregs_action.action_type not in (ActionType.MAINTAIN, ActionType.NO_ACTION):
            try:
                strategy = plan_avoidance(own, targets, self._environment, colregs_action)
            except Exception as exc:
                self.get_logger().warn(f"路径规划异常: {exc}")
        payload = self._build_decision_payload(
            own, primary_target, primary_risk, situation, colregs_action, strategy, risk_results)
        self._publish_decision(payload)
        self.get_logger().info(
            f"[决策] target={primary_target.mmsi} | CRI={primary_risk.cri:.3f} | "
            f"encounter={situation.encounter_type.value} | action={colregs_action.action_type.value}")

    def _try_return(self, own, targets):
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
            self.get_logger().warn(f"返航规划异常: {exc}")
            return_path = None
        if return_path is not None:
            self._is_avoiding = False
            payload = {
                "decision_type": "return_to_original_course",
                "own_ship_mmsi": own.mmsi,
                "return_heading": self._original_heading,
                "return_speed": self._original_speed,
                "status": "returning",
            }
            self.get_logger().info(
                f"威胁已解除，规划返航: heading={self._original_heading:.1f}deg")
        else:
            payload = {"decision_type": "hold_avoidance",
                       "own_ship_mmsi": own.mmsi, "status": "waiting_to_return"}
        self._publish_decision(payload)

    def _publish_decision(self, payload):
        try:
            msg = String()
            msg.data = json.dumps(payload, ensure_ascii=False)
            self._pub_decisions.publish(msg)
        except Exception as exc:
            self.get_logger().error(f"发布决策消息失败: {exc}")

    def _build_decision_payload(self, own, primary_target, primary_risk,
                                situation, colregs_action, strategy, all_risks):
        risk_summary = [
            {"target_mmsi": t.mmsi, "cri": round(r.cri, 4),
             "risk_level": r.risk_level.value,
             "dcpa_nm": round(r.dcpa, 4), "tcpa_min": round(r.tcpa, 2),
             "distance_nm": round(r.distance, 4)}
            for t, r in all_risks
        ]
        return {
            "decision_type": "avoidance",
            "own_ship_mmsi": own.mmsi,
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
