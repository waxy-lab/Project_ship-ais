#!/bin/bash
# 测试运行脚本
# 用于快速运行测试框架的测试

set -e

echo "========================================="
echo "运行测试框架验证"
echo "========================================="
echo ""

# 设置 PYTHONPATH
export PYTHONPATH=src:$PYTHONPATH

echo "1. 运行单元测试..."
python -m pytest src/test_framework/test/test_example.py -m unit -v

echo ""
echo "2. 运行属性测试..."
python -m pytest src/test_framework/test/test_example.py -m property -v

echo ""
echo "3. 运行集成测试..."
python -m pytest src/test_framework/test/test_example.py -m integration -v

echo ""
echo "========================================="
echo "所有测试完成！"
echo "========================================="
