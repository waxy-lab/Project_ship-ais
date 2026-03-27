#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AIS 局域网转发节点 (ais_to_lan_node)

功能：
  订阅本地 /ais/ship_states 话题，将解析后的船舶状态
  通过 MQTT 发布到对方机器（局域网同一 WiFi），
  供对方的 LLM 系统订阅使用。

启动示例：
  ros2 run ais_to_mqtt ais_to_lan --ros-args -p broker_address:=192.168.x.x
"""

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

try:
    import paho.mqtt.client as mqtt
    _HAS_MQTT = True
except ImportError:
    _HAS_MQTT = False

try:
    from ais_msgs.msg import AisShipList
    _HAS_AIS_MSG = True
except ImportError:
    _HAS_AIS_MSG = False


class AisToLanNode(Node):
    """将 /ais/ship_states 通过 MQTT 转发到局域网对方机器。"""

    def __init__(self):
        super().__init__('ais_to_lan_node')

        self.declare_parameter('broker_address', '192.168.1.100')  # <-- 改成对方IP
        self.declare_parameter('broker_port', 1883)
        self.declare_parameter('mqtt_topic', 'ais/ship_states')
        self.declare_parameter('username', '')
        self.declare_parameter('password', '')
        self.declare_parameter('client_id', 'ais_to_lan_sender')
        self.declare_parameter('publish_rate', 1.0)

        broker_address   = self.get_parameter('broker_address').value
        broker_port      = self.get_parameter('broker_port').value
        self._mqtt_topic = self.get_parameter('mqtt_topic').value
        username         = self.get_parameter('username').value
        password         = self.get_parameter('password').value
        client_id        = self.get_parameter('client_id').value
        publish_rate     = self.get_parameter('publish_rate').value

        self._min_interval      = 1.0 / max(publish_rate, 0.1)
        self._last_publish_time = 0.0
        self._pub_count         = 0
        self._error_count       = 0

        if not _HAS_MQTT:
            self.get_logger().fatal('paho-mqtt 未安装！请执行: pip install paho-mqtt')
            return

        if not _HAS_AIS_MSG:
            self.get_logger().fatal('ais_simulator 消息包不可用，请先编译 ais_simulator 包。')
            return

        self._mqtt_connected = False
        self._mqtt_client = mqtt.Client(client_id=client_id)
        self._mqtt_client.on_connect    = self._on_mqtt_connect
        self._mqtt_client.on_disconnect = self._on_mqtt_disconnect

        if username:
            self._mqtt_client.username_pw_set(username, password)

        self.get_logger().info(f'正在连接对方机器 MQTT Broker: {broker_address}:{broker_port}')
        try:
            self._mqtt_client.connect(broker_address, broker_port, keepalive=60)
            self._mqtt_client.loop_start()
        except Exception as e:
            self.get_logger().error(
                f'MQTT 连接失败: {e}\n'
                f'请检查:\n'
                f'  1. 对方机器 IP 是否正确: {broker_address}\n'
                f'  2. 对方是否已启动 MQTT Broker (mosquitto)\n'
                f'  3. 防火墙是否开放了 {broker_port} 端口\n'
                f'  4. 双方是否连接了同一个 WiFi'
            )
            return

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._sub = self.create_subscription(
            AisShipList, '/ais/ship_states', self._ship_states_callback, qos
        )

        self.get_logger().info(
            f'AIS 局域网转发节点已启动\n'
            f'  对方 Broker : {broker_address}:{broker_port}\n'
            f'  MQTT 话题   : {self._mqtt_topic}\n'
            f'  发布频率    : {publish_rate} Hz'
        )

    def _on_mqtt_connect(self, client, userdata, flags, rc):
        rc_msgs = {0:'成功',1:'协议版本错',2:'ID无效',3:'Broker不可用',4:'用户名/密码错',5:'未授权'}
        if rc == 0:
            self._mqtt_connected = True
            self.get_logger().info('已成功连接到对方机器的 MQTT Broker！')
        else:
            self._mqtt_connected = False
            self.get_logger().error(f'MQTT 连接被拒绝: {rc_msgs.get(rc, f"rc={rc}")}')

    def _on_mqtt_disconnect(self, client, userdata, rc):
        self._mqtt_connected = False
        if rc != 0:
            self.get_logger().warning(f'MQTT 连接断开 (rc={rc})，将自动重连...')

    def _ship_states_callback(self, msg):
        now = time.time()
        if now - self._last_publish_time < self._min_interval:
            return
        self._last_publish_time = now

        if not self._mqtt_connected:
            self.get_logger().warning('MQTT 未连接，跳过本次发送。', throttle_duration_sec=5.0)
            return

        ships = []
        for s in msg.ships:
            ships.append({
                'mmsi':      int(s.mmsi),
                'latitude':  round(float(s.latitude),  6),
                'longitude': round(float(s.longitude), 6),
                'heading':   round(float(s.heading),   2),
                'sog':       round(float(s.sog),       2),
                'rot':       round(float(s.rot),       2),
            })

        payload = {'timestamp': round(now, 3), 'ship_count': len(ships), 'ships': ships}

        try:
            json_str = json.dumps(payload, ensure_ascii=False)
            result = self._mqtt_client.publish(self._mqtt_topic, json_str, qos=1)
            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                self._pub_count += 1
                self.get_logger().info(
                    f'已发送 {len(ships)} 艘船数据 [{self._mqtt_topic}] (累计 {self._pub_count} 次)',
                    throttle_duration_sec=3.0
                )
            else:
                self._error_count += 1
                self.get_logger().warning(f'MQTT 发布失败 rc={result.rc}')
        except Exception as e:
            self._error_count += 1
            self.get_logger().error(f'发送数据时出错: {e}')

    def destroy_node(self):
        if hasattr(self, '_mqtt_client') and self._mqtt_client:
            self._mqtt_client.loop_stop()
            self._mqtt_client.disconnect()
            self.get_logger().info(f'已断开。成功 {self._pub_count} 次，失败 {self._error_count} 次。')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AisToLanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到 Ctrl+C，正在关闭...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()