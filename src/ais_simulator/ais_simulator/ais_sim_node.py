#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import serial  # 用于串口通信
import pyais.messages as ais_messages # 用于创建AIS消息
from pyais.encode import encode_msg   # 用于编码NMEA
import yaml
import os
from ament_index_python.packages import get_package_share_directory
import random
from shapely.geometry import Point, Polygon


class AisSimulatorNode(Node):
    #初始化相关属性
    def __init__(self):
        super().__init__('ais_simulator_node')

        # 尝试打开指定串口
        self.declare_parameter('output_port', '/dev/pts/2') 
        self.output_port_name = self.get_parameter('output_port').get_parameter_value().string_value

        # 加载模拟船只配置
        self.declare_parameter('ships_configs', 'ships_config.yaml')
        config_file = self.get_parameter('ships_configs').get_parameter_value().string_value

        self.startup_ok = False
        # 地球半径 (米)
        self.EARTH_RADIUS_METERS = 6371e3
        self.SAFETY_DISTANCE = 0.0005

        try:
            #读取船舶配置
            self.simulated_ships = self.load_ship_configs(config_file)
            if not self.simulated_ships:
                self.get_logger().error('没有加载到任何船只配置，节点将退出。')
                rclpy.shutdown()
                return
        except Exception as e:
            self.get_logger().error(f'加载船只配置时出错: {e}')
            rclpy.shutdown()
            return
        
        try: 
            # 打开串口
            # 真实的AIS设备通常使用 38400 波特率
            self.serial_port = serial.Serial(self.output_port_name, 38400, timeout=1)
            self.get_logger().info(f'成功打开虚拟串口: {self.output_port_name}')
            self.startup_ok = True
        except Exception as e:
            self.get_logger().error(f'无法打开串口 {self.output_port_name}: {e}')
            self.get_logger().error('请确保您已在另一个终端中运行 `socat` 命令！')
            rclpy.shutdown()
            return

        # 设置定时器频率 (1s)
        self.timer_period = 1.0  # (秒)
        self.timer_ = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info(f'AIS 模拟器已启动，模拟 {len(self.simulated_ships)} 艘船')

        river_coords = [
            (121.780, 31.380), # 左上
            (121.850, 31.340), # 右上
            (121.840, 31.320), # 右下
            (121.770, 31.360), # 左下
            (121.780, 31.380)  # 闭合
        ]
        self.safe_zone = Polygon(river_coords)
        self.get_logger().info('电子围栏初始化完成。')

    #计算两船之间距离
    def calculate_distance(self, lat1, lon1, lat2, lon2):
        return math.sqrt((lat1 - lat2)**2 + (lon1 - lon2)**2)

    #加载模拟船的配置
    def load_ship_configs(self, filename):
        package_name = 'ais_simulator' 
        
        try:
            # 1. 使用 ROS 2 机制找到包的共享目录
            share_directory = get_package_share_directory(package_name)
            
            # 2. 构造配置文件的完整路径
            full_path = os.path.join(share_directory, filename)

            self.get_logger().info(f'正在尝试从 ROS 2 共享目录路径: {full_path} 读取配置...')
            
            with open(full_path, 'r') as f:
                config = yaml.safe_load(f)
            
            if 'simulated_ships' in config:
                valid_ships = []
                for ship in config['simulated_ships']:
                    if all(k in ship for k in ['mmsi', 'latitude', 'longitude', 'heading', 'sog', 'rot']):
                        valid_ships.append(ship)
                    else:
                        self.get_logger().warn(f"船只配置 {ship} 缺少关键字段，已被跳过。")
                return valid_ships
            else:
                self.get_logger().error(f"配置文件 '{full_path}' 中缺少 'simulated_ships' 键。")
                return []
                
        except Exception as e:
            # 如果文件找不到，或者 YAML 解析失败，都会被捕获
            self.get_logger().error(f"无法加载或解析船舶配置文件 '{filename}'。错误: {e}")
            self.get_logger().error(f"请确保已修改 setup.py 将 '{filename}' 复制到 '{package_name}' 的共享目录！")
            raise # 重新抛出异常，让 __init__ 退出
    
    #定时器的主回调函数，每秒执行一次。
    def timer_callback(self):
        # 遍历我们内部列表中的每一艘船
        for ship_state in self.simulated_ships:
            # 动态检测碰撞
            self.avoid_collision_with_other_ships(ship_state, self.simulated_ships)
            # 更新这艘船的状态 (推算1秒后的位置)
            self.update_ship_state(ship_state)
            # 将状态编码为NMEA句子并发送
            self.generate_and_send_nmea(ship_state)

    #将信息编码成NMEA字符串发送
    def generate_and_send_nmea(self, ship_state):
        """
        核心函数：将Python字典转换为NMEA !AIVDM 字符串并发送。
        """
        try:
            # Type 1 (A类位置报告)
            msg = ais_messages.MessageType1(
                # 默认值
                msg_type=1,
                repeat=0,
                accuracy=False,  # default=0 (bool)
                maneuver=0,
                spare_1=b'',     # default=b''
                raim=False,      # default=0 (bool)
                radio=0,         # default=0

                mmsi=ship_state['mmsi'],
                status=ais_messages.NavigationStatus.UnderWayUsingEngine,
                speed=ship_state['sog'],       # 速度: 1/10 节
                lon=ship_state['longitude'],           # 经度
                lat=ship_state['latitude'],            # 纬度
                course=ship_state['heading'],   # 航向: 1/10 度
                heading=int(ship_state['heading']), # 真航向
                turn=self.convert_rot_to_ais(ship_state['rot']), # 转向率
                second=self.get_clock().now().seconds_nanoseconds()[0] % 60 # 秒
            )

            # 将消息对象编码为NMEA字符串列表
            # (一个AIS消息可能被分成多个NMEA句子)
            nmea_sentences = encode_msg(msg)

            for sentence in nmea_sentences:
                # 添加换行符 \r\n，就像真实设备一样
                full_sentence = f"{sentence}\r\n"
                # 写入串口
                self.serial_port.write(full_sentence.encode('ascii'))

            self.get_logger().info(f'MMSI {ship_state["mmsi"]}: 已发送 NMEA 句子', throttle_duration_sec=5.0)

        except Exception as e:
            self.get_logger().warn(f'编码或发送NMEA时出错: {e}')

    #辅助函数：将 度/分钟 转换为AIS的ROT字段值
    def convert_rot_to_ais(self, rot_deg_per_min):
        if rot_deg_per_min == 0:
            return 0
        # 这是一个简化的转换，真实AIS ROT字段是非线性的
        if rot_deg_per_min > 0:
            return 126
        else:
            return -126

    #船只之间互相避让,检查当前船只与其他船只的距离，并在距离过近时修改航向
    def avoid_collision_with_other_ships(self, current_ship, all_ships):
        for other_ship in all_ships:
            if current_ship is other_ship:
                continue
            
            #检测两船间的距离
            distance = self.calculate_distance(
                current_ship['latitude'], current_ship['longitude'], other_ship['latitude'], other_ship['longitude']
            )
            #距离小于安全距离则调整方向
            if distance < self.SAFETY_DISTANCE:
                self.get_logger().warn(f"{current_ship['mmsi']} 发现碰撞风险！")
                
                current_ship['heading'] = (current_ship['heading'] + 30.0) % 360
                current_ship['rot'] = 5.0
                return
            
            #恢复直航
            if current_ship['rot'] != 0.0:
                current_ship['rot'] = 0.0

    #更新位置并包含避障逻辑 + 随机扰动
    def update_ship_state(self, ship_state):
        dt = self.timer_period  # 时间间隔 (秒)

        # 模拟风浪和人为微调
        # 1. 持续的微小漂移 (模拟水流/风): -0.5度 到 +0.5度 之间随机
        drift = random.uniform(-0.5, 0.5)
        
        # 2. 偶尔的主动变向 (模拟船长调整航向): 
        # 每秒有 5% 的概率发生稍微大一点的转向 (-2 到 2 度)
        if random.random() < 0.05:
            drift += random.uniform(-2.0, 2.0)

        # 3. 计算新航向
        rot_deg_per_sec = ship_state['rot'] / 60.0
        # 将随机漂移加到航向计算中
        next_heading = ship_state['heading'] + rot_deg_per_sec * dt + drift
        next_heading = next_heading % 360.0

        # --- 以下是原来的位置计算逻辑 (保持不变) ---
        speed_mps = ship_state['sog'] * 0.514444
        distance_meters = speed_mps * dt

        if distance_meters == 0:
            return 

        bearing_rad = math.radians(next_heading)
        lat1_rad = math.radians(ship_state['latitude'])
        lon1_rad = math.radians(ship_state['longitude'])

        ang_dist = distance_meters / self.EARTH_RADIUS_METERS

        lat2_rad = math.asin(
            math.sin(lat1_rad) * math.cos(ang_dist) +
            math.cos(lat1_rad) * math.sin(ang_dist) * math.cos(bearing_rad)
        )

        lon2_rad = lon1_rad + math.atan2(
            math.sin(bearing_rad) * math.sin(ang_dist) * math.cos(lat1_rad),
            math.cos(ang_dist) - math.sin(lat1_rad) * math.sin(lat2_rad)
        )

        next_lat = math.degrees(lat2_rad)
        next_lon = math.degrees(lon2_rad)

        # --- 碰撞检测逻辑 (保持不变) ---
        next_point = Point(next_lon, next_lat)

        if self.safe_zone.contains(next_point):
            # 安全：更新位置和航向
            ship_state['latitude'] = next_lat
            ship_state['longitude'] = next_lon
            ship_state['heading'] = next_heading
        else:
            # 撞墙：执行避障
            self.get_logger().warn(f"MMSI {ship_state['mmsi']} 触碰边界！正在自动转向...")
            turn_angle = random.randint(120, 240)
            ship_state['heading'] = (ship_state['heading'] + turn_angle) % 360.0
            ship_state['rot'] = 0

def main(args=None):
    rclpy.init(args=args)
    node = AisSimulatorNode() # node 总是会被创建

    try:
        if not node.startup_ok:
            node.get_logger().error("节点因串口初始化失败而退出。")
        else:
            # 串口成功打开，正常运行
            rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info('收到 Ctrl+C，正在关闭...')

    finally:
        # 无论如何退出，都尝试销毁
        if node.startup_ok:
            # 仅在成功启动后才需要关闭串口
            node.serial_port.close()
            node.get_logger().info("串口已关闭。")

        node.destroy_node()

if __name__ == '__main__':
    main()