#!/usr/bin/env python3
"""
数据记录 ROS2 节点

订阅所有关键话题，记录AIS数据、避让决策和中间计算结果。
Requirements: 9.1-9.3
"""

import os
import json
import csv
import time
from datetime import datetime
from typing import List, Dict, Any, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .data_models import (
    AisRecord, DecisionRecord, RiskRecord, DataStore, DataExporter
)

class DataLoggerNode(Node):
    """
    数据记录 ROS2 节点
    Requirements: 9.1-9.3
    """

    def __init__(self):
        super().__init__('data_logger_node')

        # 参数
        self.declare_parameter('output_dir', '/tmp/data_logger')
        self.declare_parameter('flush_interval_sec', 60.0)
        output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        flush_interval = self.get_parameter('flush_interval_sec').get_parameter_value().double_value

        self._store = DataStore()
        self._exporter = DataExporter(output_dir=output_dir)
        self._start_time = time.time()

        # 订阅 AIS 船舶状态 Requirements: 9.1
        self._sub_ais = self.create_subscription(
            String, '/ais/ship_states',
            self._ais_callback, 10)

        # 订阅避碰决策 Requirements: 9.2
        self._sub_decisions = self.create_subscription(
            String, '/collision_avoidance/decisions',
            self._decision_callback, 10)

        # 订阅风险评估结果 Requirements: 9.3
        self._sub_risks = self.create_subscription(
            String, '/collision_avoidance/risk_assessments',
            self._risk_callback, 10)

        # 定时刷新（写入文件）
        self._flush_timer = self.create_timer(
            flush_interval, self._flush_callback)

        self.get_logger().info(
            f'DataLoggerNode 启动，输出目录: {output_dir}')

    def _ais_callback(self, msg: String):
        """处理 AIS 数据 Requirements: 9.1"""
        try:
            data = json.loads(msg.data)
            now = time.time()
            ships = data.get('ships', [])
            for ship in ships:
                record = AisRecord(
                    timestamp=now,
                    mmsi=ship.get('mmsi', 0),
                    latitude=ship.get('latitude', 0.0),
                    longitude=ship.get('longitude', 0.0),
                    heading=ship.get('heading', 0.0),
                    sog=ship.get('sog', 0.0),
                    rot=ship.get('rot', 0.0),
                )
                self._store.add_ais(record)
        except Exception as e:
            self.get_logger().warning(f'AIS数据解析失败: {e}')

    def _decision_callback(self, msg: String):
        """处理避让决策 Requirements: 9.2"""
        try:
            data = json.loads(msg.data)
            now = time.time()
            record = DecisionRecord(
                timestamp=now,
                own_mmsi=data.get('own_mmsi', 0),
                decision_type=data.get('decision_type', ''),
                target_heading=data.get('target_heading'),
                target_speed=data.get('target_speed'),
                reason=data.get('reason', ''),
                raw=msg.data,
            )
            self._store.add_decision(record)
        except Exception as e:
            self.get_logger().warning(f'决策数据解析失败: {e}')

    def _risk_callback(self, msg: String):
        """处理风险评估结果 Requirements: 9.3"""
        try:
            data = json.loads(msg.data)
            now = time.time()
            record = RiskRecord(
                timestamp=now,
                own_mmsi=data.get('own_mmsi', 0),
                target_mmsi=data.get('target_mmsi', 0),
                dcpa=data.get('dcpa', 0.0),
                tcpa=data.get('tcpa', 0.0),
                cri=data.get('cri', 0.0),
                risk_level=data.get('risk_level', 'safe'),
            )
            self._store.add_risk(record)
        except Exception as e:
            self.get_logger().warning(f'风险数据解析失败: {e}')

    def _flush_callback(self):
        """定时写入文件"""
        if self._store.ais_count == 0:
            return
        try:
            path = self._exporter.export_all_json(self._store)
            self.get_logger().info(
                f'数据已写入: {path} '
                f'(AIS:{self._store.ais_count} '
                f'决策:{self._store.decision_count} '
                f'风险:{self._store.risk_count})')
        except Exception as e:
            self.get_logger().warning(f'写入失败: {e}')

    def export_now(self) -> dict:
        """立即导出所有数据，返回文件路径字典"""
        return {
            'ais': self._exporter.export_ais_csv(self._store.get_ais_records()),
            'decisions': self._exporter.export_decisions_csv(
                self._store.get_decision_records()),
            'risks': self._exporter.export_risks_csv(
                self._store.get_risk_records()),
            'all': self._exporter.export_all_json(self._store),
        }

    @property
    def store(self) -> DataStore:
        return self._store


def main(args=None):
    rclpy.init(args=args)
    node = DataLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到 Ctrl+C，正在导出数据...')
        node.export_now()
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
