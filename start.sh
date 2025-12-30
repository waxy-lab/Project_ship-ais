#!/bin/bash

# ==========================================
# 1. 环境初始化
# ==========================================
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source $DIR/.venv/bin/activate
if [ -f "$DIR/install/setup.bash" ]; then
    source $DIR/install/setup.bash
else
    echo "错误: 未找到 install/setup.bash，请先编译。"
    exit 1
fi

# 定义固定串口路径
SIM_PORT="/tmp/ttyAIS_SIM"
READER_PORT="/tmp/ttyAIS_READ"

# 清理函数
cleanup() {
    echo -e "\n正在关闭所有节点..."
    kill $SOCAT_PID 2>/dev/null
    kill $(jobs -p) 2>/dev/null
    rm -f $SIM_PORT $READER_PORT
    echo "已退出。"
}
trap cleanup SIGINT

# ==========================================
# 2. 启动虚拟串口 (socat)
# ==========================================
echo "[1/5] 启动虚拟串口..."
# 启动 socat 并不再 sleep，而是进入循环检测
socat -d -d pty,raw,echo=0,link=$SIM_PORT pty,raw,echo=0,link=$READER_PORT &
SOCAT_PID=$!

# 核心优化：循环检测文件是否存在，存在的一瞬间立刻继续，绝不浪费时间
echo "      等待端口创建..."
while [ ! -e "$SIM_PORT" ] || [ ! -e "$READER_PORT" ]; do
    # 极短的微休眠，仅为了不占用 100% CPU，几乎等于无延迟
    sleep 0.1
done
echo "      端口已就绪！"

# ==========================================
# 3. 启动 ROS 节点 (并发启动)
# ==========================================
# 优化策略：先启动“消费者”(Reader/MQTT)，最后启动“生产者”(Simulator)
# 这样能保证模拟器发出的第一条数据就能被立刻接收，不会积压在缓存里。

echo "[2/5] 启动 AIS 读取节点 (Consumer)..."
ros2 run ais_to_mqtt ais_reader --ros-args -p port:=$READER_PORT &

echo "[3/5] 启动 数据分析节点..."
ros2 run ais_to_mqtt analyzeData &

echo "[4/5] 启动 MQTT 桥接节点..."
ros2 run ais_to_mqtt mqtt_bridge &

# 稍微等待一下 Reader 节点完成初始化（ROS节点启动通常需要几百毫秒）
# 这里的等待是为了防止 Reader 还没准备好接收，Simulator 就发完了第一波
# 如果你追求极致，这一行也可以去掉，但保留 0.5秒 是最稳妥的
sleep 0.5 

echo "[5/5] 启动 AIS 模拟器 (Producer)..."
# 模拟器最后启动，确保它开始发数据时，后面的链路都已经打通了
python3 -m ais_simulator.ais_sim_node --ros-args -p output_port:=$SIM_PORT &

echo "---------------------------------------"
echo " 系统全速运行中 | PID: $$"
echo "---------------------------------------"

wait