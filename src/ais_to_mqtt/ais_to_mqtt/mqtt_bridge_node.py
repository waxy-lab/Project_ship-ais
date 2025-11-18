#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
from std_msgs.msg import String
import json

# --- safe_float 函数已移除，因为 analyzeData 节点会处理解析 ---

class MqttBridgeNode(Node):
    def __init__(self):
        super().__init__('mqtt_bridge_node')

        # 声明 ROS 2 参数
        self.declare_parameter('broker_address', '60.205.13.156')
        self.declare_parameter('broker_port', 1883)
        self.declare_parameter('username', 'Drone/1hbahhhckc4ky')
        self.declare_parameter('password', 'tonzajuxhvmr')
        self.declare_parameter('client_id', 'waxy_ais_test')
        self.declare_parameter('mqtt_topic', 'drone/ais/data')

        # 获取参数
        broker_address = self.get_parameter('broker_address').get_parameter_value().string_value
        broker_port = self.get_parameter('broker_port').get_parameter_value().integer_value
        self.mqtt_topic_ = self.get_parameter('mqtt_topic').get_parameter_value().string_value
        username = self.get_parameter('username').get_parameter_value().string_value
        password = self.get_parameter('password').get_parameter_value().string_value
        client_id = self.get_parameter('client_id').get_parameter_value().string_value

        self.get_logger().info(f"正在连接到 MQTT Broker {broker_address}:{broker_port}")

        # --- MQTT 设置 ---
        self.mqtt_client_ = mqtt.Client(client_id)
        self.mqtt_client_.on_connect = self.on_mqtt_connect

        if username:
            self.get_logger().info(f"使用用户名: {username}")
            self.mqtt_client_.username_pw_set(username, password)
        else:
            self.get_logger().info("使用匿名连接 (无用户名)")

        # 连接mqtt服务器
        try:
            self.mqtt_client_.connect(broker_address, broker_port, 60)
            self.mqtt_client_.loop_start() # 后台线程处理 MQTT
        except Exception as e:
            self.get_logger().fatal(f"无法连接到 MQTT Broker: {e}")
            rclpy.shutdown()
            return

        # --- ROS 设置 ---
        # 订阅 /ais/realData 话题 (由 analyzeData 节点发布)
        self.subscription_ = self.create_subscription(
            String,
            '/ais/realData',
            self.ros_callback,
            10)

        self.get_logger().info("MQTT 桥接节点已启动，等待 AIS 数据...")

    # MQTT 连接回调
    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("成功连接到 MQTT Broker!")
        else:
            self.get_logger().error(f"连接 MQTT 失败，返回码 {rc}")

    # --- 【已修正】：使用您的新逻辑 (只转发，不解析) ---
    def ros_callback(self, msg):
        json_message = msg.data  # msg.data 是来自 /ais/realData 的 JSON 字符串
        self.get_logger().info(f"从 /ais/realData 收到: {json_message[:60]}...")
        
        try:
            # --- 直接发布 JSON 字符串到 MQTT ---
            if self.mqtt_client_.is_connected():
                self.mqtt_client_.publish(self.mqtt_topic_, json_message)
                self.get_logger().info(f"已作为 JSON 发布到 MQTT: {json_message[:60]}...")
            else:
                self.get_logger().warn("MQTT 未连接，JSON 消息未发送。")

        except Exception as e:
            self.get_logger().error(f"发布到 MQTT 时出错: {e}")
    # -----------------------------------------------

    def on_shutdown(self):
        self.get_logger().info("节点关闭，停止 MQTT 循环。")
        self.mqtt_client_.loop_stop()
        self.mqtt_client_.disconnect()
        
def main(args=None):
    rclpy.init(args=args)
    node = MqttBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()

if __name__ == '__main__':
    main()