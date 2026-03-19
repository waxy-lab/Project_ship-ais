#!/usr/bin/env python3
"""
可视化参数发布节点（基础版）

订阅 AIS 数据和避碰决策，实时计算并发布 DCPA、TCPA、CRI 等关键参数。
Requirements: 7.4
"""

import json
import time
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String

from ais_simulator.msg import AisShipList
from collision_avoidance.risk_assessment import assess_collision_risk
from scenario_generator.models import ShipState


def _ais_to_ship(msg):
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


class VisualizationPublisherNode(Node):
    """
    可视化参数发布节点

    实时发布以下参数供可视化使用：
    - DCPA（最近会遇距离，海里）
    - TCPA（最近会遇时间，分钟）
    - CRI（碰撞风险指数，0-1）
    - 风险等级（safe/warning/danger）
    - 本船和目标船的实时状态

    Requirements: 7.4
    """

    RISK_LEVELS = [
        (0.7, 'danger'),
        (0.3, 'warning'),
        (0.0, 'safe'),
    ]

    def __init__(self):
        super().__init__('visualization_publisher_node')

        # 参数
        self.declare_parameter('own_ship_mmsi', 0)
        self.declare_parameter('publish_rate', 2.0)   # 发布频率 Hz
        self.declare_parameter('max_range_nm', 10.0)  # 监控范围（海里）
        self._own_mmsi = self.get_parameter('own_ship_mmsi').value
        self._max_range = self.get_parameter('max_range_nm').value
        pub_rate = self.get_parameter('publish_rate').value

        self._ship_list: List[ShipState] = []
        self._own_ship: Optional[ShipState] = None
        self._last_risk_data: dict = {}

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 订阅 AIS 数据
        self._sub_ais = self.create_subscription(
            AisShipList, '/ais/ship_states', self._ais_callback, qos)

        # 发布实时风险参数（供 rviz2 / 外部可视化工具）
        self._pub_risk_params = self.create_publisher(
            String, '/visualization/risk_parameters', 10)

        # 发布船舶状态摘要
        self._pub_ship_summary = self.create_publisher(
            String, '/visualization/ship_summary', 10)

        # 发布告警
        self._pub_alerts = self.create_publisher(
            String, '/visualization/alerts', 10)

        # 定时发布
        self._timer = self.create_timer(
            1.0 / pub_rate, self._publish_loop)

        self.get_logger().info(
            f'可视化发布节点已启动 | 发布频率={pub_rate}Hz | '
            f'监控范围={self._max_range}nm')

    def _ais_callback(self, msg):
        """接收 AIS 数据"""
        ships = [_ais_to_ship(s) for s in msg.ships]
        self._ship_list = [s for s in ships if s is not None]

        if self._own_mmsi == 0:
            self._own_ship = self._ship_list[0] if self._ship_list else None
        else:
            matched = [s for s in self._ship_list if s.mmsi == self._own_mmsi]
            self._own_ship = matched[0] if matched else None

    def _get_risk_level(self, cri: float) -> str:
        for threshold, level in self.RISK_LEVELS:
            if cri >= threshold:
                return level
        return 'safe'

    def _publish_loop(self):
        """定时计算并发布参数 Requirements: 7.4"""
        if self._own_ship is None:
            return

        own = self._own_ship
        targets = [s for s in self._ship_list if s.mmsi != own.mmsi]
        now = time.time()

        # 计算每艘目标船的风险参数
        risk_list = []
        alerts = []
        for target in targets:
            try:
                result = assess_collision_risk(own, target)
                level = self._get_risk_level(result.cri)
                entry = {
                    'target_mmsi': target.mmsi,
                    'dcpa_nm': round(result.dcpa, 4),
                    'tcpa_min': round(result.tcpa, 2),
                    'cri': round(result.cri, 4),
                    'risk_level': level,
                    'distance_nm': round(result.distance, 4),
                    'timestamp': now,
                }
                risk_list.append(entry)
                if level in ('warning', 'danger'):
                    alerts.append({
                        'level': level,
                        'target_mmsi': target.mmsi,
                        'cri': round(result.cri, 4),
                        'dcpa_nm': round(result.dcpa, 4),
                        'tcpa_min': round(result.tcpa, 2),
                    })
            except Exception as e:
                self.get_logger().debug(f'风险计算失败 target={target.mmsi}: {e}')

        # 按 CRI 降序排列
        risk_list.sort(key=lambda x: x['cri'], reverse=True)

        # 发布风险参数
        risk_payload = {
            'own_mmsi': own.mmsi,
            'own_heading': round(own.heading, 1),
            'own_sog': round(own.sog, 1),
            'own_lat': round(own.latitude, 6),
            'own_lon': round(own.longitude, 6),
            'target_risks': risk_list,
            'timestamp': now,
        }
        self._last_risk_data = risk_payload
        risk_msg = String()
        risk_msg.data = json.dumps(risk_payload, ensure_ascii=False)
        self._pub_risk_params.publish(risk_msg)

        # 发布船舶摘要
        summary = {
            'total_ships': len(self._ship_list),
            'own_mmsi': own.mmsi,
            'target_count': len(targets),
            'high_risk_count': len([r for r in risk_list
                                    if r['risk_level'] == 'danger']),
            'warning_count': len([r for r in risk_list
                                   if r['risk_level'] == 'warning']),
            'timestamp': now,
        }
        summary_msg = String()
        summary_msg.data = json.dumps(summary, ensure_ascii=False)
        self._pub_ship_summary.publish(summary_msg)

        # 发布告警
        if alerts:
            alert_msg = String()
            alert_msg.data = json.dumps({
                'alerts': alerts, 'timestamp': now}, ensure_ascii=False)
            self._pub_alerts.publish(alert_msg)
            self.get_logger().warning(
                f'风险告警: {len(alerts)}个目标 | '
                f'最高CRI={alerts[0]["cri"]:.3f}')

    @property
    def last_risk_data(self) -> dict:
        """获取最近一次风险数据（用于测试）"""
        return self._last_risk_data


def main(args=None):
    rclpy.init(args=args)
    node = VisualizationPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
