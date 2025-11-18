#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from typing import Dict, Any, Optional
import math

def parse_binary_data(binary_str: str, start: int, length: int) -> int:
    """从二进制字符串中解析指定位置的整数值"""
    if start >= len(binary_str) or start + length > len(binary_str):
        return 0
    return int(binary_str[start:start+length], 2)

def sixbit_to_ascii(sixbit_str: str) -> str:
    """将6位ASCII码转换为普通ASCII字符"""
    result = []
    for char in sixbit_str:
        code = ord(char) - 48
        if code > 40:
            code -= 8
        if 0 <= code <= 63:
            if code <= 31:
                result.append(chr(code + 64))
            else:
                result.append(chr(code))
    return ''.join(result).strip()

def twos_complement(value: int, bits: int) -> int:
    """计算二进制补码"""
    if value & (1 << (bits - 1)):
        value -= 1 << bits
    return value

def _parse_position_report(binary_data: str, message_type: int) -> Dict[str, Any]:
    """解析位置报告消息 (类型1,2,3)"""
    result = {}
    
    # MMSI
    result["mmsi"] = parse_binary_data(binary_data, 8, 30)
    
    # 导航状态
    nav_status = parse_binary_data(binary_data, 38, 4)
    nav_status_dict = {
        0: "Under way using engine",
        1: "At anchor",
        2: "Not under command",
        3: "Restricted manoeuverability",
        4: "Constrained by her draught",
        5: "Moored",
        6: "Aground",
        7: "Engaged in fishing",
        8: "Under way sailing",
        15: "Not defined"
    }
    result["navigation_status"] = nav_status_dict.get(nav_status, "Unknown")
    
    # 旋转速率
    rot = parse_binary_data(binary_data, 42, 8)
    if rot == 128:
        result["rate_of_turn"] = "Not available"
    else:
        result["rate_of_turn"] = twos_complement(rot, 8)
    
    # 对地航速
    sog = parse_binary_data(binary_data, 50, 10)
    result["speed_over_ground"] = sog * 0.1 if sog < 1023 else "Not available"
    
    # 位置精度
    result["position_accuracy"] = "High" if parse_binary_data(binary_data, 60, 1) else "Low"
    
    # 经度
    longitude = twos_complement(parse_binary_data(binary_data, 61, 28), 28)
    result["longitude"] = longitude / 600000.0 if longitude != 0x6791AC0 else "Not available"
    
    # 纬度
    latitude = twos_complement(parse_binary_data(binary_data, 89, 27), 27)
    result["latitude"] = latitude / 600000.0 if latitude != 0x3412140 else "Not available"
    
    # 对地航向
    cog = parse_binary_data(binary_data, 116, 12)
    result["course_over_ground"] = cog * 0.1 if cog < 3600 else "Not available"
    
    # 真航向
    true_heading = parse_binary_data(binary_data, 128, 9)
    result["true_heading"] = true_heading if true_heading <= 359 else "Not available"
    
    # 时间戳
    timestamp = parse_binary_data(binary_data, 137, 6)
    result["timestamp"] = timestamp if timestamp <= 59 else "Not available"
    
    # 操纵指示符
    maneuver = parse_binary_data(binary_data, 143, 2)
    maneuver_dict = {
        0: "Not available",
        1: "No special maneuver",
        2: "Special maneuver"
    }
    result["maneuver_indicator"] = maneuver_dict.get(maneuver, "Unknown")
    
    result["message_description"] = f"Class A Position Report (Type {message_type})"
    
    return result

def _parse_static_voyage_data(binary_data: str) -> Dict[str, Any]:
    """解析静态和航程数据 (类型5)"""
    result = {}
    
    # MMSI
    result["mmsi"] = parse_binary_data(binary_data, 8, 30)
    
    # AIS版本
    ais_version = parse_binary_data(binary_data, 38, 2)
    result["ais_version"] = ais_version
    
    # IMO编号
    result["imo_number"] = parse_binary_data(binary_data, 40, 30)
    
    # 呼号 (内联6位ASCII解析)
    callsign_bits = binary_data[70:112]
    callsign = ""
    for i in range(0, 42, 6):
        if i + 6 <= len(callsign_bits):
            char_code = int(callsign_bits[i:i+6], 2)
            if char_code > 0:
                callsign += chr(64 + char_code) if char_code <= 26 else chr(48 + char_code - 32)
    result["callsign"] = callsign.strip()
    
    # 船名 (内联6位ASCII解析)
    shipname_bits = binary_data[112:232]
    shipname = ""
    for i in range(0, 120, 6):
        if i + 6 <= len(shipname_bits):
            char_code = int(shipname_bits[i:i+6], 2)
            if char_code > 0:
                shipname += chr(64 + char_code) if char_code <= 26 else chr(48 + char_code - 32)
    result["ship_name"] = shipname.strip()
    
    # 船舶类型
    result["ship_type"] = parse_binary_data(binary_data, 232, 8)
    
    # 尺寸
    result["dimension"] = {
        "to_bow": parse_binary_data(binary_data, 240, 9),
        "to_stern": parse_binary_data(binary_data, 249, 9),
        "to_port": parse_binary_data(binary_data, 258, 6),
        "to_starboard": parse_binary_data(binary_data, 264, 6)
    }
    
    # 定位设备类型
    epfd = parse_binary_data(binary_data, 270, 4)
    epfd_dict = {
        0: "Undefined", 1: "GPS", 2: "GLONASS", 3: "Combined GPS/GLONASS",
        4: "Loran-C", 5: "Chayka", 6: "Integrated navigation system",
        7: "Surveyed", 8: "Galileo"
    }
    result["position_fix_type"] = epfd_dict.get(epfd, "Unknown")
    
    # ETA
    eta_month = parse_binary_data(binary_data, 274, 4)
    eta_day = parse_binary_data(binary_data, 278, 5)
    eta_hour = parse_binary_data(binary_data, 283, 5)
    eta_minute = parse_binary_data(binary_data, 288, 6)
    result["eta"] = f"{eta_month:02d}-{eta_day:02d} {eta_hour:02d}:{eta_minute:02d}"
    
    # 最大静态吃水
    draught = parse_binary_data(binary_data, 294, 8)
    result["maximum_draught"] = draught * 0.1
    
    # 目的地 (内联6位ASCII解析)
    destination_bits = binary_data[302:422]
    destination = ""
    for i in range(0, 120, 6):
        if i + 6 <= len(destination_bits):
            char_code = int(destination_bits[i:i+6], 2)
            if char_code > 0:
                destination += chr(64 + char_code) if char_code <= 26 else chr(48 + char_code - 32)
    result["destination"] = destination.strip()
    
    result["message_description"] = "Static and Voyage Related Data"
    
    return result

def _parse_class_b_position_report(binary_data: str) -> Dict[str, Any]:
    """解析B类设备位置报告 (类型18)"""
    result = {}
    
    # MMSI
    result["mmsi"] = parse_binary_data(binary_data, 8, 30)
    
    # 对地航速
    sog = parse_binary_data(binary_data, 46, 10)
    result["speed_over_ground"] = sog * 0.1 if sog < 1023 else "Not available"
    
    # 位置精度
    result["position_accuracy"] = "High" if parse_binary_data(binary_data, 56, 1) else "Low"
    
    # 经度
    longitude = twos_complement(parse_binary_data(binary_data, 57, 28), 28)
    result["longitude"] = longitude / 600000.0 if longitude != 0x6791AC0 else "Not available"
    
    # 纬度
    latitude = twos_complement(parse_binary_data(binary_data, 85, 27), 27)
    result["latitude"] = latitude / 600000.0 if latitude != 0x3412140 else "Not available"
    
    # 对地航向
    cog = parse_binary_data(binary_data, 112, 12)
    result["course_over_ground"] = cog * 0.1 if cog < 3600 else "Not available"
    
    # 真航向
    true_heading = parse_binary_data(binary_data, 124, 9)
    result["true_heading"] = true_heading if true_heading <= 359 else "Not available"
    
    # 时间戳
    timestamp = parse_binary_data(binary_data, 133, 6)
    result["timestamp"] = timestamp if timestamp <= 59 else "Not available"
    
    result["message_description"] = "Class B Position Report"
    
    return result

def _parse_class_b_extended_position_report(binary_data: str) -> Dict[str, Any]:
    """解析B类扩展位置报告 (类型19)"""
    result = _parse_class_b_position_report(binary_data)
    
    # 船名 (内联6位ASCII解析)
    shipname_bits = binary_data[143:263]
    shipname = ""
    for i in range(0, 120, 6):
        if i + 6 <= len(shipname_bits):
            char_code = int(shipname_bits[i:i+6], 2)
            if char_code > 0:
                shipname += chr(64 + char_code) if char_code <= 26 else chr(48 + char_code - 32)
    result["ship_name"] = shipname.strip()
    
    # 船舶类型
    result["ship_type"] = parse_binary_data(binary_data, 263, 8)
    
    # 尺寸
    result["dimension"] = {
        "to_bow": parse_binary_data(binary_data, 271, 9),
        "to_stern": parse_binary_data(binary_data, 280, 9),
        "to_port": parse_binary_data(binary_data, 289, 6),
        "to_starboard": parse_binary_data(binary_data, 295, 6)
    }
    
    result["message_description"] = "Extended Class B Equipment Position Report"
    
    return result

def _parse_aids_to_navigation_report(binary_data: str) -> Dict[str, Any]:
    """解析助航设备报告 (类型21)"""
    result = {}
    
    # MMSI
    result["mmsi"] = parse_binary_data(binary_data, 8, 30)
    
    # 助航设备类型
    result["aid_type"] = parse_binary_data(binary_data, 38, 5)
    
    # 名称 (内联6位ASCII解析)
    name_bits = binary_data[43:163]
    name = ""
    for i in range(0, 120, 6):
        if i + 6 <= len(name_bits):
            char_code = int(name_bits[i:i+6], 2)
            if char_code > 0:
                name += chr(64 + char_code) if char_code <= 26 else chr(48 + char_code - 32)
    result["name"] = name.strip()
    
    # 位置精度
    result["position_accuracy"] = "High" if parse_binary_data(binary_data, 163, 1) else "Low"
    
    # 经度
    longitude = twos_complement(parse_binary_data(binary_data, 164, 28), 28)
    result["longitude"] = longitude / 600000.0 if longitude != 0x6791AC0 else "Not available"
    
    # 纬度
    latitude = twos_complement(parse_binary_data(binary_data, 192, 27), 27)
    result["latitude"] = latitude / 600000.0 if latitude != 0x3412140 else "Not available"
    
    # 尺寸
    result["dimension"] = {
        "to_bow": parse_binary_data(binary_data, 219, 9),
        "to_stern": parse_binary_data(binary_data, 228, 9),
        "to_port": parse_binary_data(binary_data, 237, 6),
        "to_starboard": parse_binary_data(binary_data, 243, 6)
    }
    
    result["message_description"] = "Aids-to-Navigation Report"
    
    return result

def _parse_static_data_report(binary_data: str) -> Dict[str, Any]:
    """解析静态数据报告 (类型24)"""
    result = {}
    
    # MMSI
    result["mmsi"] = parse_binary_data(binary_data, 8, 30)
    
    # 部分编号
    part_number = parse_binary_data(binary_data, 38, 2)
    result["part_number"] = part_number
    
    if part_number == 0:
        # Part A: 船名 (内联6位ASCII解析)
        name_bits = binary_data[40:160]
        name = ""
        for i in range(0, 120, 6):
            if i + 6 <= len(name_bits):
                char_code = int(name_bits[i:i+6], 2)
                if char_code > 0:
                    name += chr(64 + char_code) if char_code <= 26 else chr(48 + char_code - 32)
        result["ship_name"] = name.strip()
        result["message_description"] = "Static Data Report (Part A - Name)"
    else:
        # Part B: 船舶类型和尺寸
        result["ship_type"] = parse_binary_data(binary_data, 40, 8)
        
        # 厂商ID (内联6位ASCII解析)
        vendor_id_bits = binary_data[48:66]
        vendor_id = ""
        for i in range(0, 18, 6):
            if i + 6 <= len(vendor_id_bits):
                char_code = int(vendor_id_bits[i:i+6], 2)
                if char_code > 0:
                    vendor_id += chr(64 + char_code) if char_code <= 26 else chr(48 + char_code - 32)
        result["vendor_id"] = vendor_id.strip()
        
        # 呼号 (内联6位ASCII解析)
        callsign_bits = binary_data[90:132]
        callsign = ""
        for i in range(0, 42, 6):
            if i + 6 <= len(callsign_bits):
                char_code = int(callsign_bits[i:i+6], 2)
                if char_code > 0:
                    callsign += chr(64 + char_code) if char_code <= 26 else chr(48 + char_code - 32)
        result["callsign"] = callsign.strip()
        
        # 尺寸
        result["dimension"] = {
            "to_bow": parse_binary_data(binary_data, 132, 9),
            "to_stern": parse_binary_data(binary_data, 141, 9),
            "to_port": parse_binary_data(binary_data, 150, 6),
            "to_starboard": parse_binary_data(binary_data, 156, 6)
        }
        
        result["message_description"] = "Static Data Report (Part B - Details)"
    
    return result

def ais_parser(ais_data: str) -> Dict[str, Any]:
    """
    解析AIS NMEA 0183格式数据
    """
    # 基本NMEA字段解析
    try:
        fields = ais_data.split(',')
        if len(fields) < 6:
            return {"error": "Invalid AIS data format"}
        
        # 提取编码数据
        encoded_data = fields[5]
        if not encoded_data:
            return {"error": "No encoded data found"}
        
        # 将6位ASCII转换为二进制字符串
        binary_data = ""
        for char in encoded_data:
            ascii_val = ord(char) - 48
            if ascii_val > 40:
                ascii_val -= 8
            binary_data += format(ascii_val, '06b')
        
        # 解析消息类型
        message_type = parse_binary_data(binary_data, 0, 6)
        
        result = {
            "raw_data": ais_data,
            "message_type": message_type,
            "nmea_fields": {
                "talker": fields[0][:2],
                "sentence_type": fields[0][2:],
                "fragment_count": int(fields[1]) if fields[1] else 0,
                "fragment_number": int(fields[2]) if fields[2] else 0,
                "sequential_message_id": fields[3],
                "channel": fields[4],
                "checksum": fields[6] if len(fields) > 6 else ""
            }
        }
        
        # 根据消息类型进行具体解析
        if message_type == 1 or message_type == 2 or message_type == 3:
            result.update(_parse_position_report(binary_data, message_type))
        elif message_type == 5:
            result.update(_parse_static_voyage_data(binary_data))
        elif message_type == 18:
            result.update(_parse_class_b_position_report(binary_data))
        elif message_type == 19:
            result.update(_parse_class_b_extended_position_report(binary_data))
        elif message_type == 21:
            result.update(_parse_aids_to_navigation_report(binary_data))
        elif message_type == 24:
            result.update(_parse_static_data_report(binary_data))
        else:
            result["message_description"] = f"Message type {message_type} (not fully parsed)"
        
        return result
        
    except Exception as e:
        return {"error": f"Parsing error: {str(e)}", "raw_data": ais_data}


class AisParserNode(Node):
    """
    订阅 /ais/rawData, 解析后发布到 /ais/realData
    """
    def __init__(self):
        super().__init__('ais_parser_node')
        
        # 创建发布者，发布解析后的数据 (JSON字符串)
        self.publisher_ = self.create_publisher(
            String,
            '/ais/realData',
            10)
        
        # 创建订阅者，订阅原始AIS数据
        self.subscription_ = self.create_subscription(
            String,
            '/ais/rawData',
            self.listener_callback,
            10)
        
        self.get_logger().info('AIS 解析节点已启动.')
        self.get_logger().info('正在订阅: /ais/rawData')
        self.get_logger().info('正在发布: /ais/realData')

    def listener_callback(self, msg):
        raw_data = msg.data
        # self.get_logger().debug(f'收到原始数据: "{raw_data}"')

        # 使用 AIS 解析器
        parsed_data = ais_parser(raw_data)
        
        if 'error' in parsed_data:
            self.get_logger().warn(f'解析失败: {parsed_data["error"]} (Raw: {raw_data})')
        else:
            #发布前将raw_data和nmea_fields移除以减小消息体积
            parsed_data.pop('raw_data', None)                 #原始数据
            parsed_data.pop('nmea_fields', None)              #nmea编码的原始数据
            parsed_data.pop("message_description", None)      #消息描述
            parsed_data.pop("position_accuracy", None)        #精度
            try:
                # 将解析后的字典转换为 JSON 字符串
                json_output = json.dumps(parsed_data)
                
                # 创建并发布 String 消息
                pub_msg = String()
                pub_msg.data = json_output
                self.publisher_.publish(pub_msg)
                # self.get_logger().info(f'发布解析数据: {json_output[:50]}...')
            except Exception as e:
                self.get_logger().error(f'序列化或发布时出错: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    node = AisParserNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 停止并关闭节点
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()