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

from ais_simulator.map_loader import MapLoader

class AisSimulatorNode(Node):
    #初始化相关属性
    def __init__(self):
        super().__init__('ais_simulator_node')

        # 获取共享包的安装路径
        package_share_directory = get_package_share_directory('ais_simulator')
        default_map = os.path.join(package_share_directory, 'config', 'river.geojson')
        default_ships = os.path.join(package_share_directory, 'config', 'ships_config.yaml')

        # 尝试打开指定串口
        self.declare_parameter('output_port', '/dev/pts/2') 
        self.output_port_name = self.get_parameter('output_port').get_parameter_value().string_value

        # 加载模拟船只配置
        self.declare_parameter('ships_configs', 'ships_config.yaml')
        config_file = self.get_parameter('ships_configs').get_parameter_value().string_value

        # 加载地图配置
        self.declare_parameter('map_path', '')
        map_path_param = self.get_parameter('map_path').get_parameter_value().string_value

        self.startup_ok = False
        # 地球半径 (米)
        self.EARTH_RADIUS_METERS = 6371e3
        self.SAFETY_DISTANCE = 0.001 #110m左右的安全距离

        try:
            #读取船舶配置
            self.simulated_ships = self.load_ship_configs(default_ships)
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

        target_map = map_path_param if map_path_param else default_map

        self.safe_zone = MapLoader.load_from_geojson(target_map, self.get_logger())
        self.get_logger().info('电子围栏初始化完成。')

    #计算两船之间距离
    def calculate_distance(self, lat1, lon1, lat2, lon2):
        return math.sqrt((lat1 - lat2)**2 + (lon1 - lon2)**2)

    #加载模拟船的配置
    def load_ship_configs(self, filename):
        
        try:
            self.get_logger().info(f'正在尝试从 ROS 2 共享目录路径: {filename} 读取配置...')
            
            with open(filename, 'r') as f:
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
                self.get_logger().error(f"配置文件 '{filename}' 中缺少 'simulated_ships' 键。")
                return []
                
        except Exception as e:
            # 如果文件找不到，或者 YAML 解析失败，都会被捕获
            self.get_logger().error(f"无法加载或解析船舶配置文件 '{filename}'。错误: {e}")
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
                
                #执行避让动作
                if abs(current_ship['rot']) < 5.0:
                    turn_rot = random.choice([10.0, -10.0])
                    current_ship['rot']  = turn_rot
                current_ship['heading'] = (current_ship['heading'] + 5.0) % 360.0
                return
            
            # 如果没有风险，且船的 ROT 是碰撞避免设置的，则缓慢消除 ROT 
            if abs(current_ship['rot']) > 5.0 and abs(current_ship['rot']) < 25.0: 
                 current_ship['rot'] *= 0.8 # 缓慢减少 ROT

    def update_ship_state(self, ship_state):
        # 定义常量
        dt = self.timer_period  # 1秒
        LOOKAHEAD_TIME = 5.0    # 预警 5秒
        
        # ==========================================
        # 第一步：航点导航逻辑
        # ==========================================
        if 'waypoints' in ship_state and len(ship_state['waypoints']) > 0:
            target_lat, target_lon = ship_state['waypoints'][0]
            
            # 1. 计算距离
            dist_to_target = self.calculate_distance(
                ship_state['latitude'], ship_state['longitude'],
                target_lat, target_lon
            )
        
            # 2. 判断到达 (200米左右)
            if dist_to_target < 0.002: 
                self.get_logger().info(f"MMSI {ship_state['mmsi']} 到达航点，切换下一个！")
                ship_state['waypoints'].pop(0)
                if len(ship_state['waypoints']) > 0:
                    target_lat, target_lon = ship_state['waypoints'][0] # 更新为新目标
            
            # 3. 实时计算目标方位角 (Target Heading)
            if len(ship_state['waypoints']) > 0:
                ship_state['target_heading'] = self.calculate_bearing(
                    ship_state['latitude'], ship_state['longitude'],
                    target_lat, target_lon
                )

        # ==========================================
        # 第二步：基础导航控制
        # ==========================================

        if 'target_heading' in ship_state:
            diff = self.get_heading_diff(ship_state['target_heading'], ship_state['heading'])
            
            # 简单的 P 控制器：偏差越大，转得越快
            # 限制最大转向率为 10.0 度/分钟 (模拟正常航行时的转向)
            nav_rot = max(-10.0, min(10.0, diff * 0.5))
            
            # 应用导航指令 (稍后可能会被避障逻辑覆盖)
            ship_state['rot'] = nav_rot

        # ==========================================
        # 第三步：物理状态更新预备
        # ==========================================
        # 1. 随机扰动 (模拟风浪)
        drift = random.uniform(-0.5, 0.5)
        if random.random() < 0.05: drift += random.uniform(-2.0, 2.0)

        # 2. 准备计算参数
        # 注意：这里我们使用刚才计算出的 nav_rot 来更新航向
        rot_deg_per_sec = ship_state['rot'] / 60.0
        next_heading = (ship_state['heading'] + rot_deg_per_sec * dt + drift) % 360.0
        
        speed_mps = ship_state['sog'] * 0.514444
        dist_1s = speed_mps * dt
        dist_5s = speed_mps * LOOKAHEAD_TIME

        if dist_1s == 0: return 

        # 转换弧度供计算
        bearing_rad = math.radians(next_heading)
        lat_rad = math.radians(ship_state['latitude'])
        lon_rad = math.radians(ship_state['longitude'])
        ang_dist_1s = dist_1s / self.EARTH_RADIUS_METERS
        ang_dist_5s = dist_5s / self.EARTH_RADIUS_METERS

        # 3. 计算未来位置
        # 1秒后 (用于撞墙检测)
        next_lat, next_lon = self.calculate_new_position(lat_rad, lon_rad, bearing_rad, ang_dist_1s)
        # 5秒后 (用于预警)
        future_lat, future_lon = self.calculate_new_position(lat_rad, lon_rad, bearing_rad, ang_dist_5s)

        # ==========================================
        # 第四步：双层避障逻辑 (安全 > 导航)
        # ==========================================
        warning_point = Point(future_lon, future_lat)
        immediate_point = Point(next_lon, next_lat)
        
        # --- [层级 A] 预警检查 (5秒) ---
        if not self.safe_zone.contains(warning_point):
            # 发现 5秒后可能出界，且当前不是在做剧烈转向 (ROT < 5.0)
            # 这意味着：如果“正常导航”会导致出界，我们就“覆盖”它
            if abs(ship_state['rot']) < 5.0: 
                soft_turn = random.choice([3.0, -3.0]) # 柔和避让
                ship_state['rot'] = soft_turn 
                self.get_logger().info(f"MMSI {ship_state['mmsi']}: 预判触岸，覆盖导航指令，执行柔和转向。")

        # --- [层级 B] 立即碰撞检查 (1秒) ---
        if self.safe_zone.contains(immediate_point):
            # --- 安全 ---
            # 提交位置更新
            ship_state['latitude'] = next_lat
            ship_state['longitude'] = next_lon
            ship_state['heading'] = next_heading
            
            # 如果是因为刚才紧急避让导致 ROT 很大，现在安全了，需要快速衰减 ROT，
            # 让控制权交还给“航点导航逻辑”
            if abs(ship_state['rot']) > 10.0:
                 ship_state['rot'] *= 0.8
            
        else:
            # --- 危险 (撞墙) ---
            self.get_logger().warn(f"MMSI {ship_state['mmsi']} 撞墙警告！执行紧急避让。")
            
            # 强制大角度转向，完全接管控制权
            if abs(ship_state['rot']) < 10.0:
                hard_turn = random.choice([15.0, -15.0])
                ship_state['rot'] = hard_turn 
            
            # 不更新位置 (原地停一帧旋转)

    #辅助函数1：将 度/分钟 转换为AIS的ROT字段值
    def convert_rot_to_ais(self, rot_deg_per_min):
        if rot_deg_per_min == 0:
            return 0
        # 这是一个简化的转换，真实AIS ROT字段是非线性的
        if rot_deg_per_min > 0:
            return 126
        else:
            return -126
    
    #辅助函数2：根据球面距离和航向计算新位置
    def calculate_new_position(self, lat1_rad, lon1_rad, bearing_rad, ang_dist):
        lat2_rad = math.asin(
            math.sin(lat1_rad) * math.cos(ang_dist) +
            math.cos(lat1_rad) * math.sin(ang_dist) * math.cos(bearing_rad)
        )
        lon2_rad = lon1_rad + math.atan2(
            math.sin(bearing_rad) * math.sin(ang_dist) * math.cos(lat1_rad),
            math.cos(ang_dist) - math.sin(lat1_rad) * math.sin(lat2_rad)
        )
        return math.degrees(lat2_rad), math.degrees(lon2_rad)

    #辅助函数3：计算方位角
    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        diff_lon_rad = math.radians(lon2 - lon1)

        y = math.sin(diff_lon_rad) * math.cos(lat2_rad)
        x = math.cos(lat1_rad) * math.sin(lat2_rad) - \
            math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(diff_lon_rad)
        
        bearing = math.degrees(math.atan2(y, x))
        return (bearing + 360) % 360
    
    # 辅助函数4：计算两个角度的最小差值
    def get_heading_diff(self, target, current):
        diff = (target - current + 180) % 360 - 180
        return diff

        
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