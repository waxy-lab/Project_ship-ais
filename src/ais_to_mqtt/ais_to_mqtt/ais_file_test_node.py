#!/usr/bin/env python3
# (建议在文件顶部添加这一行)

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class AisFileTestNode(Node):
    def __init__(self):
        super().__init__('ais_file_test_node')

        #declare_parameter启动时没收到关于'ais_file_path'的指示，使用'AIS_1'作为默认值
        self.declare_parameter('ais_file_path', '/home/waxy/ais_ws/AIS_1.txt')
        self.declare_parameter('publish_rate_sec', 1.0)

        #get_parameter获取'ais_file_path'当前值
        file_path = self.get_parameter('ais_file_path').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate_sec').get_parameter_value().double_value

        #创建发布者
        self.publisher = self.create_publisher(String, '/ais/logData', 10)

        self.get_logger().info(f"正在从文件中读取AIS数据:{file_path}")

        try:
            with open(file_path, 'r') as f:
                self.lines = f.readlines()

            self.line_index = 0
            self.total_lines = len(self.lines)

            if self.total_lines == 0:
                self.get_logger().error("文件为空！")
                # <--- 改动: 不调用 shutdown，而是抛出异常
                raise RuntimeError("AIS test file is empty.")
            
            #创建定时器循环发布
            self.timer = self.create_timer(publish_rate,self.publish_line)
            # <--- 改动 (Bug 修复): self.get_logger 后面需要加 ()
            self.get_logger().info(f"启动测试... 以每{publish_rate}秒一条的速率发布{self.total_lines}条数据。")

        except FileNotFoundError:
            self.get_logger().fatal(f"找不到文件：{file_path}")
            # <--- 改动: 不调用 shutdown，而是重新抛出异常
            raise
        except Exception as e:
            # <--- 改动 (Bug 修复): 字符串需要 f"" 才能格式化
            self.get_logger().error(f"读取文件错误：{e}")
            # <--- 改动: 不调用 shutdown，而是重新抛出异常
            raise

    #从文件中读取每行数据并发布
    def publish_line(self):
        if self.line_index >= self.total_lines:
            self.get_logger().info("已到达文件末尾，重新循环...")
            self.line_index = 0

        #获取当前行并去除首尾空白
        line = self.lines[self.line_index].strip()

        if line:
            msg = String()
            msg.data = line
            self.publisher.publish(msg)
            self.get_logger().info(f"发布 (line {self.line_index + 1}/{self.total_lines}): {line[:50]}...") # 打印前50个字符

        self.line_index += 1

def main(args=None):
    rclpy.init(args=args)
    
    # <--- 改动: 初始化 node 为 None
    node = None
    try:
        # <--- 改动: 节点创建放在 try 块中
        node = AisFileTestNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass # 用户按下了 Ctrl+C
    except Exception as e:
        # <--- 改动: 捕获 __init__ 中抛出的异常
        if node:
            node.get_logger().fatal(f"节点运行时出现未处理异常: {e}")
        else:
            print(f"创建节点时出错: {e}") # 节点可能还未创建成功，只能用 print
    finally:
        # <--- 改动: 销毁前检查 node 是否存在
        if node:
            node.destroy_node()
        
        # <--- 改动: 关闭前检查 rclpy 是否还在运行
        # 这是防止 "Context must be initialized" 错误的最安全方法
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()