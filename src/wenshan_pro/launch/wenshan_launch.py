#!/usr/bin/env python3
"""
文山项目一键启动文件
启动顺序：AIS读取器 -> AIS解析器 -> MQTT桥接
（AIS模拟器需单独用 python3 -m 方式启动）
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/pts/9',
        description='AIS读取串口号（socat 输出的第二个端口）'
    )

    # AIS 读取器节点
    ais_reader = Node(
        package='ais_to_mqtt',
        executable='ais_reader',
        name='ais_reader',
        parameters=[{'port': LaunchConfiguration('port')}],
        output='screen'
    )

    # AIS 解析器节点
    ais_parser = Node(
        package='ais_to_mqtt',
        executable='analyzeData',
        name='ais_parser',
        output='screen'
    )

    # MQTT 桥接节点
    mqtt_bridge = Node(
        package='ais_to_mqtt',
        executable='mqtt_bridge',
        name='mqtt_bridge',
        output='screen'
    )

    return LaunchDescription([
        port_arg,
        ais_reader,
        ais_parser,
        mqtt_bridge,
    ])
