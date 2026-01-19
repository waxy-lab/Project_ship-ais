#!/bin/bash
# 依赖安装脚本
# 水路仿真测试框架与智能避碰系统

set -e  # 遇到错误立即退出

echo "========================================="
echo "安装系统依赖..."
echo "========================================="

# 检查是否在虚拟环境中
if [ -z "$VIRTUAL_ENV" ]; then
    echo "警告: 未检测到虚拟环境，建议在虚拟环境中安装"
    read -p "是否继续? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# 安装Python依赖
echo "安装Python依赖包..."
pip install --upgrade pip
pip install -r requirements.txt

# 安装额外的测试依赖
echo "安装测试框架依赖..."
pip install lark -q

echo ""
echo "========================================="
echo "检查ROS2环境..."
echo "========================================="

# 检查ROS2是否已安装
if [ -z "$ROS_DISTRO" ]; then
    echo "警告: 未检测到ROS2环境变量"
    echo "请确保已安装ROS2并source了setup.bash"
    echo "例如: source /opt/ros/humble/setup.bash"
else
    echo "检测到ROS2发行版: $ROS_DISTRO"
fi

echo ""
echo "========================================="
echo "创建必要的目录..."
echo "========================================="

# 创建配置目录
mkdir -p config
mkdir -p test_results
mkdir -p logs

# 创建测试目录
mkdir -p src/scenario_generator/test
mkdir -p src/collision_avoidance/test
mkdir -p src/test_framework/test

echo ""
echo "========================================="
echo "依赖安装完成!"
echo "========================================="
echo ""
echo "下一步:"
echo "1. 确保已source ROS2环境: source /opt/ros/humble/setup.bash"
echo "2. 编译工作空间: colcon build"
echo "3. source工作空间: source install/setup.bash"
echo "4. 运行测试: pytest"
echo ""
