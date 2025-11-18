#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class AisSerialReaderNode(Node):
    def __init__(self):
        # 1. 初始化 Node，节点名称为 'ais_serial_reader'
        super().__init__('ais_serial_reader')

        # 2. 声明 ROS 2 参数 (替代 @Value)
        #    这会创建可以从命令行或 launch 文件中覆盖的参数
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 38400)

        # 3. 获取参数值
        port_name = self.get_parameter('port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud').get_parameter_value().integer_value

        # 4. 创建发布者 (Publisher)
        #    它将向 '/ais_rawData' 主题发布 std_msgs/String 类型的消息
        self.publisher_ = self.create_publisher(String, '/ais/rawData', 10)

        self.get_logger().info(f"尝试打开串口: {port_name} @ {baud_rate} b/s")

        # 5. 配置并打开串口
        try:
            self.ser = serial.Serial(
                port=port_name,
                baudrate=baud_rate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1.0  # 读取超时1秒
            )
            self.get_logger().info(f"成功打开端口: {self.ser.name}")
            self.get_logger().info("开始读取AIS数据...")
            self.get_logger().info("--------------------------------------------------")

        except serial.SerialException as e:
            self.get_logger().fatal(f"无法打开串口 {port_name}: {e}")
            self.get_logger().fatal("请检查:")
            self.get_logger().fatal(f"1. {port_name} 端口是否存在 (使用 'ls /dev/tty*' 检查)")
            self.get_logger().fatal("2. 端口是否被其他程序占用")
            self.get_logger().fatal("3. 是否有读写权限 (可能需要 'sudo chmod a+rw /dev/ttyUSB0')")
            # 设置一个标志，使 run() 知道不应继续
            self.ser = None

    def run(self):
        #接收ais数据并发布到/rawData
        if self.ser is None or not self.ser.is_open:
            self.get_logger().error("串口未初始化或未打开。节点将不执行。")
            return

        # 6. 循环读取和发布 (替代 SerialPortDataListener)
        #    rclpy.ok() 在节点被关闭时 (例如 Ctrl+C) 会返回 False
        while rclpy.ok():
            try:
                # readline() 会读取直到遇到 '\n'
                line_bytes = self.ser.readline()
                if not line_bytes:
                    # (超时，返回空字节，继续循环)
                    continue
                # 解码为 US-ASCII 字符串
                line_str = line_bytes.decode('ascii', 'ignore').strip()

                if line_str and (line_str.startswith('!') or 'AIVDM' in line_str or 'AIVDO' in line_str):
                    # 6a. Log 到 ROS 2 的日志系统
                    self.get_logger().info(f"接收到原始数据: {line_str}")
                    # 6b. 发布！
                    msg = String()
                    msg.data = line_str
                    self.publisher_.publish(msg)

                    self.get_logger().info("=> 成功发布一条原始 AIS 消息到 /ais/rawData")
                    self.get_logger().info("------------------------------")            

            except serial.SerialException as e:
                self.get_logger().error(f"串口读取错误: {e}")
                break # 出现错误，退出循环
            except Exception as e:
                self.get_logger().error(f"发生意外错误: {e}")
  
    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info(f"串口 {self.ser.name} 已关闭。")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    ais_reader_node = AisSerialReaderNode()

    try:
        ais_reader_node.run()
    except KeyboardInterrupt:
        ais_reader_node.get_logger().info('用户中断 (KeyboardInterrupt)')
    finally:
        ais_reader_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()