"""
碰撞风险评估模块测试

测试 DCPA/TCPA 计算函数的正确性
"""

import pytest
import math
import sys
import os
from hypothesis import given, strategies as st, assume

# 添加路径以导入模块
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../scenario_generator'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../test_framework'))

from scenario_generator.models import ShipState
from collision_avoidance.risk_assessment import (
    calculate_dcpa_tcpa,
    calculate_cri,
    calculate_relative_bearing,
    calculate_bearing_risk,
    assess_collision_risk,
    determine_risk_level,
    RiskLevel,
    SAFE_DISTANCE,
    SAFE_TIME,
    RISK_THRESHOLD_WARNING,
    RISK_THRESHOLD_DANGER
)
from test_framework.strategies import ship_state_strategy


class TestDCPATCPACalculation:
    """DCPA/TCPA 计算函数测试"""
    
    def test_head_on_collision_course(self):
        """测试对遇碰撞航线"""
        # 两船相向航行，会在中点相遇
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=90.0,  # 向东
            sog=10.0
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=0.0,
            longitude=1.0,  # 1度 = 60海里（在赤道附近）
            heading=270.0,  # 向西
            sog=10.0
        )
        
        dcpa, tcpa = calculate_dcpa_tcpa(own_ship, target_ship)
        
        # 对遇碰撞航线，DCPA应该接近0
        assert dcpa < 0.1, f"对遇碰撞航线DCPA应接近0，实际为{dcpa}"
        
        # TCPA应该是正数（未来会相遇）
        assert tcpa > 0, f"TCPA应为正数，实际为{tcpa}"
        
        # 大约需要3小时相遇（60海里 / 20节相对速度 = 3小时 = 180分钟）
        assert 170 < tcpa < 190, f"TCPA应约为180分钟，实际为{tcpa}"
    
    def test_parallel_same_speed(self):
        """测试平行同速航行"""
        # 两船平行航行，相同速度
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=90.0,  # 向东
            sog=10.0
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=0.1,  # 北侧0.1度（约6海里）
            longitude=0.0,
            heading=90.0,  # 向东
            sog=10.0
        )
        
        dcpa, tcpa = calculate_dcpa_tcpa(own_ship, target_ship)
        
        # 平行同速，DCPA应等于当前距离（约6海里）
        assert 5.5 < dcpa < 6.5, f"平行同速DCPA应约为6海里，实际为{dcpa}"
        
        # TCPA应为无穷大（永不相遇）
        assert math.isinf(tcpa), f"平行同速TCPA应为无穷大，实际为{tcpa}"
    
    def test_ships_moving_apart(self):
        """测试两船正在远离"""
        # 两船背向航行
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=270.0,  # 向西
            sog=10.0
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=0.0,
            longitude=0.1,  # 东侧0.1度
            heading=90.0,  # 向东
            sog=10.0
        )
        
        dcpa, tcpa = calculate_dcpa_tcpa(own_ship, target_ship)
        
        # 正在远离，DCPA应等于当前距离
        current_distance = 0.1 * 60  # 约6海里
        assert abs(dcpa - current_distance) < 0.5, f"远离时DCPA应等于当前距离{current_distance}，实际为{dcpa}"
        
        # TCPA应为0（最近点已过）
        assert tcpa == 0.0, f"远离时TCPA应为0，实际为{tcpa}"
    
    def test_crossing_scenario(self):
        """测试交叉相遇场景"""
        # 两船垂直交叉
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=0.0,  # 向北
            sog=10.0
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=0.0,
            longitude=0.1,  # 东侧0.1度
            heading=270.0,  # 向西
            sog=10.0
        )
        
        dcpa, tcpa = calculate_dcpa_tcpa(own_ship, target_ship)
        
        # DCPA应该大于0（不会碰撞）
        assert dcpa > 0, f"交叉场景DCPA应大于0，实际为{dcpa}"
        
        # TCPA应该是正数
        assert tcpa > 0, f"交叉场景TCPA应为正数，实际为{tcpa}"
    
    def test_stationary_ships(self):
        """测试静止船舶"""
        # 两船都静止
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=0.0,
            sog=0.0
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=0.1,
            longitude=0.1,
            heading=0.0,
            sog=0.0
        )
        
        dcpa, tcpa = calculate_dcpa_tcpa(own_ship, target_ship)
        
        # 静止时，DCPA等于当前距离
        expected_distance = math.sqrt(0.1**2 + 0.1**2) * 60  # 约8.5海里
        assert abs(dcpa - expected_distance) < 0.5, f"静止时DCPA应约为{expected_distance}，实际为{dcpa}"
        
        # TCPA应为无穷大
        assert math.isinf(tcpa), f"静止时TCPA应为无穷大，实际为{tcpa}"
    
    def test_overtaking_scenario(self):
        """测试追越场景"""
        # 后船追赶前船
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=90.0,  # 向东
            sog=15.0  # 更快
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=0.0,
            longitude=0.1,  # 前方0.1度
            heading=90.0,  # 向东
            sog=10.0  # 较慢
        )
        
        dcpa, tcpa = calculate_dcpa_tcpa(own_ship, target_ship)
        
        # 追越时，DCPA应该很小（会接近）
        assert dcpa < 1.0, f"追越场景DCPA应很小，实际为{dcpa}"
        
        # TCPA应该是正数（未来会接近）
        assert tcpa > 0, f"追越场景TCPA应为正数，实际为{tcpa}"
    
    def test_parallel_different_speed(self):
        """测试平行不同速航行"""
        # 两船平行航行，但速度不同
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=90.0,  # 向东
            sog=10.0
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=0.1,  # 北侧0.1度（约6海里）
            longitude=0.0,
            heading=90.0,  # 向东
            sog=15.0  # 更快
        )
        
        dcpa, tcpa = calculate_dcpa_tcpa(own_ship, target_ship)
        
        # 平行不同速，DCPA应等于当前距离（约6海里）
        assert 5.5 < dcpa < 6.5, f"平行不同速DCPA应约为6海里，实际为{dcpa}"
        
        # 平行不同速时，目标船会超过本船，TCPA应为0（最近点已过或正在经过）
        # 因为目标船更快且在侧面，相对运动会导致它们错开
        assert tcpa == 0.0, f"平行不同速TCPA应为0，实际为{tcpa}"
    
    def test_perpendicular_crossing(self):
        """测试垂直交叉（90度）"""
        # 两船垂直交叉，会在交叉点相遇
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=0.0,  # 向北
            sog=10.0
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=0.0,
            longitude=0.1,  # 东侧0.1度（约6海里）
            heading=270.0,  # 向西
            sog=10.0
        )
        
        dcpa, tcpa = calculate_dcpa_tcpa(own_ship, target_ship)
        
        # DCPA应该大于0（不会完全碰撞，因为不在同一直线上）
        assert dcpa >= 0, f"垂直交叉DCPA应大于等于0，实际为{dcpa}"
        
        # TCPA应该是正数
        assert tcpa > 0, f"垂直交叉TCPA应为正数，实际为{tcpa}"
    
    def test_one_ship_stationary(self):
        """测试一艘船静止"""
        # 本船运动，目标船静止
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=90.0,  # 向东
            sog=10.0
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=0.0,
            longitude=0.1,  # 东侧0.1度
            heading=0.0,
            sog=0.0  # 静止
        )
        
        dcpa, tcpa = calculate_dcpa_tcpa(own_ship, target_ship)
        
        # DCPA应该很小（会接近静止船）
        assert dcpa < 1.0, f"接近静止船DCPA应很小，实际为{dcpa}"
        
        # TCPA应该是正数
        assert tcpa > 0, f"接近静止船TCPA应为正数，实际为{tcpa}"
    
    def test_same_position_different_heading(self):
        """测试相同位置不同航向（边界情况）"""
        # 两船在相同位置，但航向不同
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=0.0,  # 向北
            sog=10.0
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=0.0,
            longitude=0.0,  # 相同位置
            heading=90.0,  # 向东
            sog=10.0
        )
        
        dcpa, tcpa = calculate_dcpa_tcpa(own_ship, target_ship)
        
        # 当前位置重合，DCPA应该接近0
        assert dcpa < 0.1, f"相同位置DCPA应接近0，实际为{dcpa}"
        
        # TCPA应该为0（当前就是最近点）
        assert tcpa == 0.0, f"相同位置TCPA应为0，实际为{tcpa}"
    
    def test_very_close_ships(self):
        """测试非常接近的船舶（边界情况）"""
        # 两船非常接近（0.01海里 ≈ 18米）
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=90.0,  # 向东
            sog=10.0
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=0.0,
            longitude=0.01/60,  # 0.01海里
            heading=270.0,  # 向西
            sog=10.0
        )
        
        dcpa, tcpa = calculate_dcpa_tcpa(own_ship, target_ship)
        
        # DCPA应该接近0（会碰撞）
        assert dcpa < 0.1, f"非常接近的船DCPA应接近0，实际为{dcpa}"
        
        # TCPA应该很小（很快相遇）
        assert 0 < tcpa < 1, f"非常接近的船TCPA应很小，实际为{tcpa}"
    
    def test_very_far_ships(self):
        """测试非常远的船舶"""
        # 两船相距很远（10度 ≈ 600海里）
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=90.0,  # 向东
            sog=10.0
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=0.0,
            longitude=10.0,  # 10度（约600海里）
            heading=270.0,  # 向西
            sog=10.0
        )
        
        dcpa, tcpa = calculate_dcpa_tcpa(own_ship, target_ship)
        
        # DCPA应该接近0（对遇碰撞航线）
        assert dcpa < 1.0, f"远距离对遇DCPA应接近0，实际为{dcpa}"
        
        # TCPA应该很大（需要很长时间相遇）
        assert tcpa > 1000, f"远距离对遇TCPA应很大，实际为{tcpa}"
    
    def test_high_speed_ships(self):
        """测试高速船舶"""
        # 两船高速对遇（30节）
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=90.0,  # 向东
            sog=30.0  # 高速
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=0.0,
            longitude=1.0,  # 1度（约60海里）
            heading=270.0,  # 向西
            sog=30.0  # 高速
        )
        
        dcpa, tcpa = calculate_dcpa_tcpa(own_ship, target_ship)
        
        # DCPA应该接近0（对遇碰撞航线）
        assert dcpa < 0.1, f"高速对遇DCPA应接近0，实际为{dcpa}"
        
        # TCPA应该较小（高速相遇快）
        # 60海里 / 60节相对速度 = 1小时 = 60分钟
        assert 50 < tcpa < 70, f"高速对遇TCPA应约为60分钟，实际为{tcpa}"
    
    def test_low_speed_ships(self):
        """测试低速船舶"""
        # 两船低速对遇（2节）
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=90.0,  # 向东
            sog=2.0  # 低速
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=0.0,
            longitude=0.1,  # 0.1度（约6海里）
            heading=270.0,  # 向西
            sog=2.0  # 低速
        )
        
        dcpa, tcpa = calculate_dcpa_tcpa(own_ship, target_ship)
        
        # DCPA应该接近0（对遇碰撞航线）
        assert dcpa < 0.1, f"低速对遇DCPA应接近0，实际为{dcpa}"
        
        # TCPA应该较大（低速相遇慢）
        # 6海里 / 4节相对速度 = 1.5小时 = 90分钟
        assert 80 < tcpa < 100, f"低速对遇TCPA应约为90分钟，实际为{tcpa}"
    
    def test_oblique_approach(self):
        """测试斜向接近"""
        # 两船斜向接近（45度角）
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=45.0,  # 东北方向
            sog=10.0
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=0.1,
            longitude=0.1,
            heading=225.0,  # 西南方向（相向）
            sog=10.0
        )
        
        dcpa, tcpa = calculate_dcpa_tcpa(own_ship, target_ship)
        
        # DCPA应该接近0（相向航线）
        assert dcpa < 0.5, f"斜向接近DCPA应很小，实际为{dcpa}"
        
        # TCPA应该是正数
        assert tcpa > 0, f"斜向接近TCPA应为正数，实际为{tcpa}"
    
    def test_near_miss_scenario(self):
        """测试险些相遇场景"""
        # 两船几乎相遇但会错过
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=90.0,  # 向东
            sog=10.0
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=0.01,  # 北侧0.01度（约0.6海里）
            longitude=0.1,  # 东侧0.1度
            heading=270.0,  # 向西
            sog=10.0
        )
        
        dcpa, tcpa = calculate_dcpa_tcpa(own_ship, target_ship)
        
        # DCPA应该很小但不为0（险些相遇）
        assert 0.5 < dcpa < 1.0, f"险些相遇DCPA应在0.5-1.0海里，实际为{dcpa}"
        
        # TCPA应该是正数
        assert tcpa > 0, f"险些相遇TCPA应为正数，实际为{tcpa}"
    
    def test_both_ships_stationary(self):
        """测试两船都静止的边界情况"""
        # 两船都静止，相距一定距离
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=0.0,
            sog=0.0
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=0.05,  # 北侧0.05度（约3海里）
            longitude=0.05,  # 东侧0.05度（约3海里）
            heading=0.0,
            sog=0.0
        )
        
        dcpa, tcpa = calculate_dcpa_tcpa(own_ship, target_ship)
        
        # 静止时，DCPA应等于当前距离
        expected_distance = math.sqrt(0.05**2 + 0.05**2) * 60  # 约4.24海里
        assert abs(dcpa - expected_distance) < 0.5, \
            f"两船静止时DCPA应约为{expected_distance:.2f}海里，实际为{dcpa:.2f}"
        
        # TCPA应为无穷大（永不相遇）
        assert math.isinf(tcpa), f"两船静止时TCPA应为无穷大，实际为{tcpa}"
    
    def test_one_ship_stationary(self):
        """测试一船静止、一船运动的情况"""
        # 本船静止
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=0.0,
            sog=0.0
        )
        # 目标船向本船驶来
        target_ship = ShipState(
            mmsi=987654321,
            latitude=0.0,
            longitude=0.1,  # 东侧0.1度（约6海里）
            heading=270.0,  # 向西
            sog=10.0
        )
        
        dcpa, tcpa = calculate_dcpa_tcpa(own_ship, target_ship)
        
        # DCPA应该很小（目标船会接近本船）
        assert dcpa < 0.5, f"一船静止时DCPA应很小，实际为{dcpa}"
        
        # TCPA应该是正数
        assert tcpa > 0, f"一船静止时TCPA应为正数，实际为{tcpa}"
        
        # 大约需要36分钟到达（6海里 / 10节 = 0.6小时 = 36分钟）
        assert 30 < tcpa < 42, f"TCPA应约为36分钟，实际为{tcpa}"
    
    def test_same_position(self):
        """测试两船位置完全相同的边界情况"""
        # 两船在同一位置（碰撞状态）
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=0.0,
            sog=10.0
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=0.0,
            longitude=0.0,
            heading=90.0,
            sog=10.0
        )
        
        dcpa, tcpa = calculate_dcpa_tcpa(own_ship, target_ship)
        
        # 位置相同时，DCPA应为0
        assert dcpa < 0.01, f"位置相同时DCPA应接近0，实际为{dcpa}"
        
        # TCPA应为0（已经在最近点）
        assert tcpa == 0.0, f"位置相同时TCPA应为0，实际为{tcpa}"
    
    def test_parallel_opposite_direction(self):
        """测试平行反向航行"""
        # 两船平行但反向航行
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=90.0,  # 向东
            sog=10.0
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=0.1,  # 北侧0.1度（约6海里）
            longitude=0.0,
            heading=270.0,  # 向西
            sog=10.0
        )
        
        dcpa, tcpa = calculate_dcpa_tcpa(own_ship, target_ship)
        
        # 平行反向，DCPA应等于当前距离（约6海里）
        assert 5.5 < dcpa < 6.5, f"平行反向DCPA应约为6海里，实际为{dcpa}"
        
        # 平行反向时，由于浮点运算精度，TCPA可能是一个非常小的正数或0
        # 实际上它们不会相遇，但数值计算可能给出接近0的值
        assert tcpa < 0.01, f"平行反向TCPA应接近0，实际为{tcpa}"
    
    def test_perpendicular_miss(self):
        """测试垂直航行但不会相遇的情况"""
        # 两船垂直航行，但会错过
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=0.0,  # 向北
            sog=10.0
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=-0.1,  # 南侧0.1度
            longitude=0.1,  # 东侧0.1度
            heading=270.0,  # 向西
            sog=10.0
        )
        
        dcpa, tcpa = calculate_dcpa_tcpa(own_ship, target_ship)
        
        # DCPA应大于0（不会碰撞）
        assert dcpa > 0, f"垂直错过时DCPA应大于0，实际为{dcpa}"
        
        # 由于目标船在南侧且向西行驶，而本船向北，它们正在远离
        # TCPA应为0（最近点已过）
        assert tcpa == 0.0, f"垂直错过时TCPA应为0（已错过），实际为{tcpa}"
    
    def test_very_slow_approach(self):
        """测试非常缓慢接近的情况"""
        # 两船非常缓慢地相向航行
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=90.0,  # 向东
            sog=1.0  # 很慢
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=0.0,
            longitude=1.0,  # 远距离（60海里）
            heading=270.0,  # 向西
            sog=1.0  # 很慢
        )
        
        dcpa, tcpa = calculate_dcpa_tcpa(own_ship, target_ship)
        
        # DCPA应该很小（会相遇）
        assert dcpa < 0.5, f"缓慢接近时DCPA应很小，实际为{dcpa}"
        
        # TCPA应该很大（需要很长时间）
        # 60海里 / 2节 = 30小时 = 1800分钟
        assert tcpa > 1700, f"缓慢接近时TCPA应很大，实际为{tcpa}"
    
    def test_high_speed_approach(self):
        """测试高速接近的情况"""
        # 两船高速相向航行
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=90.0,  # 向东
            sog=25.0  # 高速
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=0.0,
            longitude=0.1,  # 6海里
            heading=270.0,  # 向西
            sog=25.0  # 高速
        )
        
        dcpa, tcpa = calculate_dcpa_tcpa(own_ship, target_ship)
        
        # DCPA应该很小（会相遇）
        assert dcpa < 0.5, f"高速接近时DCPA应很小，实际为{dcpa}"
        
        # TCPA应该很小（很快相遇）
        # 6海里 / 50节 = 0.12小时 = 7.2分钟
        assert tcpa < 10, f"高速接近时TCPA应很小，实际为{tcpa}"
    
    def test_slight_angle_approach(self):
        """测试小角度接近的情况"""
        # 两船以小角度接近
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=85.0,  # 接近向东
            sog=10.0
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=0.0,
            longitude=0.1,
            heading=275.0,  # 接近向西
            sog=10.0
        )
        
        dcpa, tcpa = calculate_dcpa_tcpa(own_ship, target_ship)
        
        # DCPA应该较小但不为0
        assert 0 < dcpa < 2.0, f"小角度接近时DCPA应较小，实际为{dcpa}"
        
        # TCPA应该是正数
        assert tcpa > 0, f"小角度接近时TCPA应为正数，实际为{tcpa}"
    
    def test_already_passed(self):
        """测试已经错过最近点的情况"""
        # 两船已经错过，正在远离
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=90.0,  # 向东
            sog=10.0
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=0.1,  # 北侧
            longitude=-0.1,  # 西侧（已经过去）
            heading=90.0,  # 也向东
            sog=8.0  # 较慢
        )
        
        dcpa, tcpa = calculate_dcpa_tcpa(own_ship, target_ship)
        
        # DCPA应等于当前距离（正在远离）
        current_distance = math.sqrt(0.1**2 + 0.1**2) * 60
        assert abs(dcpa - current_distance) < 1.0, \
            f"已错过时DCPA应约为当前距离{current_distance:.2f}，实际为{dcpa:.2f}"
        
        # TCPA应为0（最近点已过）
        assert tcpa == 0.0, f"已错过时TCPA应为0，实际为{tcpa}"


# ============================================================================
# 属性测试 (Property-Based Tests)
# ============================================================================

class TestDCPATCPAProperties:
    """DCPA/TCPA 计算的属性测试
    
    使用 Hypothesis 进行基于属性的测试，验证通用正确性
    """
    
    @given(
        ship1_data=ship_state_strategy(
            lat_range=(-60.0, 60.0),
            lon_range=(-150.0, 150.0),
            speed_range=(1.0, 25.0)
        ),
        ship2_data=ship_state_strategy(
            lat_range=(-60.0, 60.0),
            lon_range=(-150.0, 150.0),
            speed_range=(1.0, 25.0)
        )
    )
    def test_property_dcpa_tcpa_symmetry(self, ship1_data, ship2_data):
        """
        Feature: maritime-collision-avoidance, Property 6: DCPA/TCPA计算的对称性
        
        **Validates: Requirements 4.1**
        
        属性：对于任意两艘船舶状态，从船A视角计算的DCPA/TCPA应等于从船B视角计算的DCPA/TCPA
        
        这个属性验证了 DCPA/TCPA 计算的对称性，即：
        - calculate_dcpa_tcpa(ship1, ship2) == calculate_dcpa_tcpa(ship2, ship1)
        
        对称性是碰撞风险评估的基本要求，因为两船之间的最近会遇距离和时间
        应该是相同的，不应该依赖于从哪个视角进行计算。
        """
        # 过滤掉位置完全相同的船舶（会导致除零）
        lat_diff = abs(ship1_data['latitude'] - ship2_data['latitude'])
        lon_diff = abs(ship1_data['longitude'] - ship2_data['longitude'])
        assume(lat_diff > 0.001 or lon_diff > 0.001)
        
        # 确保两艘船的 MMSI 不同
        if ship1_data['mmsi'] == ship2_data['mmsi']:
            # 如果相同，修改第二艘船的 MMSI
            if ship2_data['mmsi'] < 999999999:
                ship2_data['mmsi'] = ship2_data['mmsi'] + 1
            else:
                ship2_data['mmsi'] = 100000000  # 回到最小值
        
        # 创建船舶状态对象
        ship1 = ShipState(**ship1_data)
        ship2 = ShipState(**ship2_data)
        
        # 从两个视角计算 DCPA/TCPA
        dcpa1, tcpa1 = calculate_dcpa_tcpa(ship1, ship2)
        dcpa2, tcpa2 = calculate_dcpa_tcpa(ship2, ship1)
        
        # 验证对称性：DCPA 应该相等
        # 使用相对误差容忍度，因为浮点运算可能有微小差异
        if math.isfinite(dcpa1) and math.isfinite(dcpa2):
            # 对于有限值，使用相对误差
            if dcpa1 > 0.01:  # 避免除以接近零的数
                relative_error = abs(dcpa1 - dcpa2) / dcpa1
                assert relative_error < 0.01, \
                    f"DCPA对称性验证失败: dcpa1={dcpa1:.6f}, dcpa2={dcpa2:.6f}, 相对误差={relative_error:.6f}"
            else:
                # 对于接近零的值，使用绝对误差
                absolute_error = abs(dcpa1 - dcpa2)
                assert absolute_error < 0.01, \
                    f"DCPA对称性验证失败: dcpa1={dcpa1:.6f}, dcpa2={dcpa2:.6f}, 绝对误差={absolute_error:.6f}"
        else:
            # 对于无穷大，两者都应该是无穷大
            assert math.isinf(dcpa1) == math.isinf(dcpa2), \
                f"DCPA对称性验证失败: dcpa1={dcpa1}, dcpa2={dcpa2} (无穷大不匹配)"
        
        # 验证对称性：TCPA 应该相等
        if math.isfinite(tcpa1) and math.isfinite(tcpa2):
            # 对于有限值，使用相对误差
            if abs(tcpa1) > 0.1:  # 避免除以接近零的数
                relative_error = abs(tcpa1 - tcpa2) / abs(tcpa1)
                assert relative_error < 0.01, \
                    f"TCPA对称性验证失败: tcpa1={tcpa1:.6f}, tcpa2={tcpa2:.6f}, 相对误差={relative_error:.6f}"
            else:
                # 对于接近零的值，使用绝对误差
                absolute_error = abs(tcpa1 - tcpa2)
                assert absolute_error < 0.1, \
                    f"TCPA对称性验证失败: tcpa1={tcpa1:.6f}, tcpa2={tcpa2:.6f}, 绝对误差={absolute_error:.6f}"
        else:
            # 对于无穷大，两者都应该是无穷大
            assert math.isinf(tcpa1) == math.isinf(tcpa2), \
                f"TCPA对称性验证失败: tcpa1={tcpa1}, tcpa2={tcpa2} (无穷大不匹配)"


if __name__ == '__main__':
    pytest.main([__file__, '-v'])


# ============================================================================
# CRI 计算测试
# ============================================================================

class TestCRICalculation:
    """CRI（碰撞风险指数）计算函数测试"""
    
    def test_high_risk_scenario(self):
        """测试高风险场景：DCPA小、TCPA小"""
        # 紧急避让场景
        dcpa = 0.3  # 0.3海里
        tcpa = 3.0  # 3分钟
        bearing = 0.0  # 正前方
        speed_ratio = 1.2
        
        cri = calculate_cri(dcpa, tcpa, bearing, speed_ratio)
        
        # 高风险场景，CRI应该大于危险阈值（0.7）
        assert cri > 0.7, f"高风险场景CRI应大于0.7，实际为{cri}"
        assert cri <= 1.0, f"CRI应小于等于1.0，实际为{cri}"
    
    def test_low_risk_scenario(self):
        """测试低风险场景：DCPA大、TCPA大"""
        # 安全场景
        dcpa = 5.0  # 5海里
        tcpa = 30.0  # 30分钟
        bearing = 90.0  # 侧方
        speed_ratio = 0.8
        
        cri = calculate_cri(dcpa, tcpa, bearing, speed_ratio)
        
        # 低风险场景，CRI应该小于预警阈值（0.5）
        assert cri < 0.5, f"低风险场景CRI应小于0.5，实际为{cri}"
        assert cri >= 0.0, f"CRI应大于等于0.0，实际为{cri}"
    
    def test_medium_risk_scenario(self):
        """测试中等风险场景"""
        # 预警场景
        dcpa = 1.0  # 1海里
        tcpa = 8.0  # 8分钟
        bearing = 30.0  # 前方偏右
        speed_ratio = 1.0
        
        cri = calculate_cri(dcpa, tcpa, bearing, speed_ratio)
        
        # 中等风险场景，CRI应该在预警和危险阈值之间
        assert 0.5 <= cri < 0.7, f"中等风险场景CRI应在0.5-0.7之间，实际为{cri}"
    
    def test_zero_dcpa(self):
        """测试DCPA为0的情况（碰撞航线）"""
        dcpa = 0.0
        tcpa = 5.0
        bearing = 0.0
        speed_ratio = 1.0
        
        cri = calculate_cri(dcpa, tcpa, bearing, speed_ratio)
        
        # DCPA为0，距离因子应该是1.0，CRI应该很高
        assert cri > 0.6, f"DCPA为0时CRI应很高，实际为{cri}"
    
    def test_infinite_tcpa(self):
        """测试TCPA为无穷大的情况（平行航行）"""
        dcpa = 3.0
        tcpa = float('inf')
        bearing = 90.0
        speed_ratio = 1.0
        
        cri = calculate_cri(dcpa, tcpa, bearing, speed_ratio)
        
        # TCPA为无穷大，时间因子应该是0，CRI应该较低
        assert cri < 0.5, f"TCPA为无穷大时CRI应较低，实际为{cri}"
    
    def test_negative_tcpa(self):
        """测试TCPA为负数的情况（正在远离）"""
        dcpa = 2.0
        tcpa = -10.0
        bearing = 180.0  # 后方
        speed_ratio = 0.8
        
        cri = calculate_cri(dcpa, tcpa, bearing, speed_ratio)
        
        # TCPA为负数，时间因子应该是0，CRI应该较低
        assert cri < 0.4, f"TCPA为负数时CRI应较低，实际为{cri}"
    
    def test_bearing_effect_front(self):
        """测试方位对CRI的影响：正前方"""
        dcpa = 1.5
        tcpa = 10.0
        bearing_front = 0.0  # 正前方
        bearing_side = 90.0  # 侧方
        speed_ratio = 1.0
        
        cri_front = calculate_cri(dcpa, tcpa, bearing_front, speed_ratio)
        cri_side = calculate_cri(dcpa, tcpa, bearing_side, speed_ratio)
        
        # 正前方的风险应该高于侧方
        assert cri_front > cri_side, \
            f"正前方CRI({cri_front})应大于侧方CRI({cri_side})"
    
    def test_bearing_effect_rear(self):
        """测试方位对CRI的影响：正后方"""
        dcpa = 1.5
        tcpa = 10.0
        bearing_front = 0.0  # 正前方
        bearing_rear = 180.0  # 正后方
        speed_ratio = 1.0
        
        cri_front = calculate_cri(dcpa, tcpa, bearing_front, speed_ratio)
        cri_rear = calculate_cri(dcpa, tcpa, bearing_rear, speed_ratio)
        
        # 正前方的风险应该高于正后方
        assert cri_front > cri_rear, \
            f"正前方CRI({cri_front})应大于正后方CRI({cri_rear})"
    
    def test_speed_ratio_effect(self):
        """测试速度比对CRI的影响"""
        dcpa = 1.5
        tcpa = 10.0
        bearing = 45.0
        speed_ratio_low = 0.5
        speed_ratio_high = 1.5
        
        cri_low = calculate_cri(dcpa, tcpa, bearing, speed_ratio_low)
        cri_high = calculate_cri(dcpa, tcpa, bearing, speed_ratio_high)
        
        # 速度比高的风险应该略高
        assert cri_high > cri_low, \
            f"高速度比CRI({cri_high})应大于低速度比CRI({cri_low})"
    
    def test_cri_range(self):
        """测试CRI值始终在0-1范围内"""
        # 测试多种极端情况
        test_cases = [
            (0.0, 0.0, 0.0, 2.0),      # 极端高风险
            (10.0, 100.0, 180.0, 0.0), # 极端低风险
            (0.5, 5.0, 0.0, 1.0),      # 中等风险
            (2.0, float('inf'), 90.0, 1.0),  # 无穷TCPA
            (1.0, -5.0, 180.0, 0.5),   # 负TCPA
        ]
        
        for dcpa, tcpa, bearing, speed_ratio in test_cases:
            cri = calculate_cri(dcpa, tcpa, bearing, speed_ratio)
            assert 0.0 <= cri <= 1.0, \
                f"CRI应在0-1范围内，实际为{cri} (dcpa={dcpa}, tcpa={tcpa})"
    
    def test_distance_factor_dominance(self):
        """测试距离因子的主导作用（权重40%）"""
        # 距离很近但其他因素一般
        dcpa_close = 0.2
        tcpa_long = 20.0
        bearing = 90.0
        speed_ratio = 1.0
        
        cri_close = calculate_cri(dcpa_close, tcpa_long, bearing, speed_ratio)
        
        # 即使TCPA较长，由于DCPA很小，CRI仍应较高
        assert cri_close > 0.5, \
            f"距离很近时CRI应较高，实际为{cri_close}"
    
    def test_time_factor_importance(self):
        """测试时间因子的重要性（权重30%）"""
        # 距离一般但时间很短
        dcpa_medium = 1.5
        tcpa_short = 2.0
        bearing = 90.0
        speed_ratio = 1.0
        
        cri_short = calculate_cri(dcpa_medium, tcpa_short, bearing, speed_ratio)
        
        # 时间很短时，CRI应该较高
        assert cri_short > 0.4, \
            f"时间很短时CRI应较高，实际为{cri_short}"


class TestBearingRisk:
    """方位因子计算测试"""
    
    def test_bearing_risk_front(self):
        """测试正前方的方位因子"""
        bearing = 0.0
        factor = calculate_bearing_risk(bearing)
        
        # 正前方应该是最危险的，因子接近1
        assert 0.9 < factor <= 1.0, \
            f"正前方方位因子应接近1，实际为{factor}"
    
    def test_bearing_risk_side(self):
        """测试侧方的方位因子"""
        bearing = 90.0
        factor = calculate_bearing_risk(bearing)
        
        # 侧方应该是中等危险，因子约0.5
        assert 0.4 < factor < 0.6, \
            f"侧方方位因子应约为0.5，实际为{factor}"
    
    def test_bearing_risk_rear(self):
        """测试正后方的方位因子"""
        bearing = 180.0
        factor = calculate_bearing_risk(bearing)
        
        # 正后方应该是较低危险，因子接近0
        assert 0.0 <= factor < 0.1, \
            f"正后方方位因子应接近0，实际为{factor}"
    
    def test_bearing_risk_symmetry(self):
        """测试方位因子的左右对称性"""
        bearing_left = -45.0
        bearing_right = 45.0
        
        factor_left = calculate_bearing_risk(bearing_left)
        factor_right = calculate_bearing_risk(bearing_right)
        
        # 左右对称，因子应该相等
        assert abs(factor_left - factor_right) < 0.01, \
            f"左右方位因子应相等，左={factor_left}，右={factor_right}"
    
    def test_bearing_risk_range(self):
        """测试方位因子始终在0-1范围内"""
        for bearing in range(-180, 181, 10):
            factor = calculate_bearing_risk(float(bearing))
            assert 0.0 <= factor <= 1.0, \
                f"方位因子应在0-1范围内，bearing={bearing}，factor={factor}"


class TestRelativeBearing:
    """相对方位计算测试"""
    
    def test_relative_bearing_front(self):
        """测试目标船在正前方"""
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=0.0,  # 向北
            sog=10.0
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=0.1,  # 北侧
            longitude=0.0,
            heading=0.0,
            sog=10.0
        )
        
        bearing = calculate_relative_bearing(own_ship, target_ship)
        
        # 目标船在正前方，相对方位应接近0
        assert abs(bearing) < 5.0, \
            f"正前方相对方位应接近0，实际为{bearing}"
    
    def test_relative_bearing_right(self):
        """测试目标船在右侧"""
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=0.0,  # 向北
            sog=10.0
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=0.0,
            longitude=0.1,  # 东侧（右侧）
            heading=0.0,
            sog=10.0
        )
        
        bearing = calculate_relative_bearing(own_ship, target_ship)
        
        # 目标船在右侧，相对方位应接近90度
        assert 85.0 < bearing < 95.0, \
            f"右侧相对方位应接近90，实际为{bearing}"
    
    def test_relative_bearing_left(self):
        """测试目标船在左侧"""
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=0.0,  # 向北
            sog=10.0
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=0.0,
            longitude=-0.1,  # 西侧（左侧）
            heading=0.0,
            sog=10.0
        )
        
        bearing = calculate_relative_bearing(own_ship, target_ship)
        
        # 目标船在左侧，相对方位应接近-90度
        assert -95.0 < bearing < -85.0, \
            f"左侧相对方位应接近-90，实际为{bearing}"
    
    def test_relative_bearing_rear(self):
        """测试目标船在正后方"""
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=0.0,  # 向北
            sog=10.0
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=-0.1,  # 南侧（后方）
            longitude=0.0,
            heading=0.0,
            sog=10.0
        )
        
        bearing = calculate_relative_bearing(own_ship, target_ship)
        
        # 目标船在正后方，相对方位应接近±180度
        assert abs(abs(bearing) - 180.0) < 5.0, \
            f"正后方相对方位应接近±180，实际为{bearing}"


class TestAssessCollisionRisk:
    """综合风险评估函数测试"""
    
    def test_assess_high_risk(self):
        """测试高风险场景的综合评估"""
        # 对遇碰撞航线
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=90.0,  # 向东
            sog=10.0
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=0.0,
            longitude=0.05,  # 3海里
            heading=270.0,  # 向西
            sog=12.0
        )
        
        risk = assess_collision_risk(own_ship, target_ship)
        
        # 验证风险等级
        assert risk.risk_level in [RiskLevel.WARNING, RiskLevel.DANGER], \
            f"高风险场景应触发预警或危险，实际为{risk.risk_level}"
        
        # 验证CRI值
        assert risk.cri > 0.5, f"高风险场景CRI应大于0.5，实际为{risk.cri}"
        
        # 验证DCPA很小
        assert risk.dcpa < 1.0, f"对遇碰撞航线DCPA应很小，实际为{risk.dcpa}"
        
        # 验证TCPA为正数
        assert risk.tcpa > 0, f"TCPA应为正数，实际为{risk.tcpa}"
    
    def test_assess_low_risk(self):
        """测试低风险场景的综合评估"""
        # 平行航行
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=90.0,  # 向东
            sog=10.0
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=0.1,  # 北侧6海里
            longitude=0.0,
            heading=90.0,  # 向东
            sog=10.0
        )
        
        risk = assess_collision_risk(own_ship, target_ship)
        
        # 验证风险等级
        assert risk.risk_level == RiskLevel.SAFE, \
            f"低风险场景应为安全，实际为{risk.risk_level}"
        
        # 验证CRI值
        assert risk.cri < 0.5, f"低风险场景CRI应小于0.5，实际为{risk.cri}"
    
    def test_assess_risk_data_completeness(self):
        """测试风险评估结果的数据完整性"""
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=90.0,
            sog=10.0
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=0.0,
            longitude=0.1,
            heading=270.0,
            sog=12.0
        )
        
        risk = assess_collision_risk(own_ship, target_ship)
        
        # 验证所有字段都有值
        assert risk.dcpa is not None
        assert risk.tcpa is not None
        assert risk.cri is not None
        assert risk.risk_level is not None
        assert risk.relative_bearing is not None
        assert risk.distance is not None
        assert risk.speed_ratio is not None
        
        # 验证数值范围
        assert risk.dcpa >= 0
        assert 0.0 <= risk.cri <= 1.0
        assert -180.0 <= risk.relative_bearing <= 180.0
        assert risk.distance >= 0
        assert risk.speed_ratio >= 0


# ============================================================================
# CRI 属性测试 (Property-Based Tests)
# ============================================================================

class TestCRIProperties:
    """CRI（碰撞风险指数）计算的属性测试
    
    使用 Hypothesis 进行基于属性的测试，验证CRI计算的通用正确性
    """
    
    @given(
        dcpa_base=st.floats(min_value=0.5, max_value=5.0, allow_nan=False, allow_infinity=False),
        tcpa_base=st.floats(min_value=5.0, max_value=30.0, allow_nan=False, allow_infinity=False),
        bearing=st.floats(min_value=-180.0, max_value=180.0, allow_nan=False, allow_infinity=False),
        speed_ratio=st.floats(min_value=0.5, max_value=2.0, allow_nan=False, allow_infinity=False),
        reduction_factor=st.floats(min_value=0.1, max_value=0.9, allow_nan=False, allow_infinity=False)
    )
    def test_property_cri_monotonicity_dcpa(self, dcpa_base, tcpa_base, bearing, speed_ratio, reduction_factor):
        """
        Feature: maritime-collision-avoidance, Property 10: CRI值的单调性
        
        **Validates: Requirements 4.2**
        
        属性：对于任意两艘船舶状态，当DCPA减小时，计算出的CRI值应增加（风险上升）
        
        这个属性验证了 CRI 计算的单调性，即：
        - 当 DCPA 减小时，CRI 应该增加
        - 这反映了距离越近，碰撞风险越高的基本原则
        
        测试策略：
        1. 生成一个基准DCPA值
        2. 计算减小后的DCPA值（乘以reduction_factor）
        3. 验证减小DCPA后的CRI值大于基准CRI值
        """
        # 过滤掉可能导致数值不稳定的极端情况
        assume(dcpa_base > 0.1)
        assume(tcpa_base > 1.0)
        
        # 计算基准CRI
        cri_base = calculate_cri(dcpa_base, tcpa_base, bearing, speed_ratio)
        
        # 减小DCPA（距离更近）
        dcpa_reduced = dcpa_base * reduction_factor
        
        # 计算减小DCPA后的CRI
        cri_reduced = calculate_cri(dcpa_reduced, tcpa_base, bearing, speed_ratio)
        
        # 验证单调性：DCPA减小时，CRI应该增加
        assert cri_reduced > cri_base, \
            f"DCPA减小时CRI应增加: dcpa_base={dcpa_base:.3f}, dcpa_reduced={dcpa_reduced:.3f}, " \
            f"cri_base={cri_base:.3f}, cri_reduced={cri_reduced:.3f}"
        
        # 验证CRI值在有效范围内
        assert 0.0 <= cri_base <= 1.0, f"基准CRI应在0-1范围内: {cri_base}"
        assert 0.0 <= cri_reduced <= 1.0, f"减小DCPA后的CRI应在0-1范围内: {cri_reduced}"
    
    @given(
        dcpa_base=st.floats(min_value=0.5, max_value=5.0, allow_nan=False, allow_infinity=False),
        tcpa_base=st.floats(min_value=5.0, max_value=30.0, allow_nan=False, allow_infinity=False),
        bearing=st.floats(min_value=-180.0, max_value=180.0, allow_nan=False, allow_infinity=False),
        speed_ratio=st.floats(min_value=0.5, max_value=2.0, allow_nan=False, allow_infinity=False),
        reduction_factor=st.floats(min_value=0.1, max_value=0.9, allow_nan=False, allow_infinity=False)
    )
    def test_property_cri_monotonicity_tcpa(self, dcpa_base, tcpa_base, bearing, speed_ratio, reduction_factor):
        """
        Feature: maritime-collision-avoidance, Property 10: CRI值的单调性
        
        **Validates: Requirements 4.2**
        
        属性：对于任意两艘船舶状态，当TCPA减小时，计算出的CRI值应增加（风险上升）
        
        这个属性验证了 CRI 计算的单调性，即：
        - 当 TCPA 减小时，CRI 应该增加
        - 这反映了时间越短，碰撞风险越高的基本原则
        
        测试策略：
        1. 生成一个基准TCPA值
        2. 计算减小后的TCPA值（乘以reduction_factor）
        3. 验证减小TCPA后的CRI值大于基准CRI值
        """
        # 过滤掉可能导致数值不稳定的极端情况
        assume(dcpa_base > 0.1)
        assume(tcpa_base > 1.0)
        
        # 计算基准CRI
        cri_base = calculate_cri(dcpa_base, tcpa_base, bearing, speed_ratio)
        
        # 减小TCPA（时间更短）
        tcpa_reduced = tcpa_base * reduction_factor
        
        # 计算减小TCPA后的CRI
        cri_reduced = calculate_cri(dcpa_base, tcpa_reduced, bearing, speed_ratio)
        
        # 验证单调性：TCPA减小时，CRI应该增加
        assert cri_reduced > cri_base, \
            f"TCPA减小时CRI应增加: tcpa_base={tcpa_base:.3f}, tcpa_reduced={tcpa_reduced:.3f}, " \
            f"cri_base={cri_base:.3f}, cri_reduced={cri_reduced:.3f}"
        
        # 验证CRI值在有效范围内
        assert 0.0 <= cri_base <= 1.0, f"基准CRI应在0-1范围内: {cri_base}"
        assert 0.0 <= cri_reduced <= 1.0, f"减小TCPA后的CRI应在0-1范围内: {cri_reduced}"
    
    @given(
        dcpa_base=st.floats(min_value=0.5, max_value=5.0, allow_nan=False, allow_infinity=False),
        tcpa_base=st.floats(min_value=5.0, max_value=30.0, allow_nan=False, allow_infinity=False),
        bearing=st.floats(min_value=-180.0, max_value=180.0, allow_nan=False, allow_infinity=False),
        speed_ratio=st.floats(min_value=0.5, max_value=2.0, allow_nan=False, allow_infinity=False),
        dcpa_reduction=st.floats(min_value=0.1, max_value=0.9, allow_nan=False, allow_infinity=False),
        tcpa_reduction=st.floats(min_value=0.1, max_value=0.9, allow_nan=False, allow_infinity=False)
    )
    def test_property_cri_monotonicity_both(self, dcpa_base, tcpa_base, bearing, speed_ratio, 
                                           dcpa_reduction, tcpa_reduction):
        """
        Feature: maritime-collision-avoidance, Property 10: CRI值的单调性
        
        **Validates: Requirements 4.2**
        
        属性：对于任意两艘船舶状态，当DCPA和TCPA同时减小时，CRI值应显著增加
        
        这个属性验证了 CRI 计算的复合单调性，即：
        - 当 DCPA 和 TCPA 同时减小时，CRI 应该增加更多
        - 这反映了距离和时间双重压力下，碰撞风险急剧上升
        
        测试策略：
        1. 生成基准DCPA和TCPA值
        2. 同时减小DCPA和TCPA
        3. 验证CRI值显著增加
        """
        # 过滤掉可能导致数值不稳定的极端情况
        assume(dcpa_base > 0.1)
        assume(tcpa_base > 1.0)
        
        # 计算基准CRI
        cri_base = calculate_cri(dcpa_base, tcpa_base, bearing, speed_ratio)
        
        # 同时减小DCPA和TCPA
        dcpa_reduced = dcpa_base * dcpa_reduction
        tcpa_reduced = tcpa_base * tcpa_reduction
        
        # 计算减小后的CRI
        cri_reduced = calculate_cri(dcpa_reduced, tcpa_reduced, bearing, speed_ratio)
        
        # 验证单调性：DCPA和TCPA同时减小时，CRI应该增加
        assert cri_reduced > cri_base, \
            f"DCPA和TCPA同时减小时CRI应增加: " \
            f"dcpa: {dcpa_base:.3f} -> {dcpa_reduced:.3f}, " \
            f"tcpa: {tcpa_base:.3f} -> {tcpa_reduced:.3f}, " \
            f"cri: {cri_base:.3f} -> {cri_reduced:.3f}"
        
        # 验证CRI值在有效范围内
        assert 0.0 <= cri_base <= 1.0, f"基准CRI应在0-1范围内: {cri_base}"
        assert 0.0 <= cri_reduced <= 1.0, f"减小后的CRI应在0-1范围内: {cri_reduced}"
    
    @given(
        dcpa=st.floats(min_value=0.1, max_value=10.0, allow_nan=False, allow_infinity=False),
        tcpa=st.floats(min_value=1.0, max_value=60.0, allow_nan=False, allow_infinity=False),
        bearing=st.floats(min_value=-180.0, max_value=180.0, allow_nan=False, allow_infinity=False),
        speed_ratio=st.floats(min_value=0.1, max_value=3.0, allow_nan=False, allow_infinity=False)
    )
    def test_property_cri_range(self, dcpa, tcpa, bearing, speed_ratio):
        """
        Feature: maritime-collision-avoidance, Property 10: CRI值的单调性
        
        **Validates: Requirements 4.2**
        
        属性：对于任意有效的输入参数，CRI值应始终在[0, 1]范围内
        
        这个属性验证了 CRI 计算的边界约束，确保：
        - CRI 值永远不会超出 [0, 1] 范围
        - 这是风险指数的基本要求
        """
        # 计算CRI
        cri = calculate_cri(dcpa, tcpa, bearing, speed_ratio)
        
        # 验证CRI在有效范围内
        assert 0.0 <= cri <= 1.0, \
            f"CRI应在0-1范围内: cri={cri:.3f}, " \
            f"dcpa={dcpa:.3f}, tcpa={tcpa:.3f}, bearing={bearing:.1f}, speed_ratio={speed_ratio:.2f}"
    
    @given(
        dcpa=st.floats(min_value=0.1, max_value=5.0, allow_nan=False, allow_infinity=False),
        tcpa=st.floats(min_value=1.0, max_value=30.0, allow_nan=False, allow_infinity=False),
        speed_ratio=st.floats(min_value=0.5, max_value=2.0, allow_nan=False, allow_infinity=False)
    )
    def test_property_cri_bearing_symmetry(self, dcpa, tcpa, speed_ratio):
        """
        Feature: maritime-collision-avoidance, Property 10: CRI值的单调性
        
        **Validates: Requirements 4.2**
        
        属性：对于相同的DCPA、TCPA和速度比，左右对称的方位应产生相同的CRI值
        
        这个属性验证了方位因子的对称性：
        - bearing = 45° 和 bearing = -45° 应产生相同的CRI
        - 这反映了左右方位的风险是对称的
        """
        bearing_positive = 45.0
        bearing_negative = -45.0
        
        # 计算正方位的CRI
        cri_positive = calculate_cri(dcpa, tcpa, bearing_positive, speed_ratio)
        
        # 计算负方位的CRI
        cri_negative = calculate_cri(dcpa, tcpa, bearing_negative, speed_ratio)
        
        # 验证对称性：左右方位的CRI应该相等
        assert abs(cri_positive - cri_negative) < 0.001, \
            f"左右对称方位的CRI应相等: " \
            f"cri(+45°)={cri_positive:.4f}, cri(-45°)={cri_negative:.4f}"
    
    @given(
        dcpa=st.floats(min_value=0.1, max_value=5.0, allow_nan=False, allow_infinity=False),
        tcpa=st.floats(min_value=1.0, max_value=30.0, allow_nan=False, allow_infinity=False),
        speed_ratio=st.floats(min_value=0.5, max_value=2.0, allow_nan=False, allow_infinity=False)
    )
    def test_property_cri_bearing_front_higher_than_rear(self, dcpa, tcpa, speed_ratio):
        """
        Feature: maritime-collision-avoidance, Property 10: CRI值的单调性
        
        **Validates: Requirements 4.2**
        
        属性：对于相同的DCPA、TCPA和速度比，正前方的CRI应高于正后方
        
        这个属性验证了方位因子的合理性：
        - 正前方（0°）的风险应高于正后方（180°）
        - 这反映了前方目标比后方目标更危险
        """
        bearing_front = 0.0
        bearing_rear = 180.0
        
        # 计算正前方的CRI
        cri_front = calculate_cri(dcpa, tcpa, bearing_front, speed_ratio)
        
        # 计算正后方的CRI
        cri_rear = calculate_cri(dcpa, tcpa, bearing_rear, speed_ratio)
        
        # 验证：正前方的CRI应高于正后方
        assert cri_front > cri_rear, \
            f"正前方CRI应高于正后方: " \
            f"cri_front={cri_front:.4f}, cri_rear={cri_rear:.4f}"


# ============================================================================
# 风险阈值判定测试
# ============================================================================

class TestRiskLevelDetermination:
    """风险等级判定函数测试
    
    测试 determine_risk_level() 函数的正确性
    Requirements: 4.3, 4.4
    """
    
    def test_safe_level_low_cri(self):
        """测试安全等级：CRI远低于预警阈值"""
        cri = 0.2  # 远低于0.5
        risk_level = determine_risk_level(cri)
        
        assert risk_level == RiskLevel.SAFE, \
            f"CRI={cri}应判定为SAFE，实际为{risk_level}"
    
    def test_safe_level_at_boundary(self):
        """测试安全等级：CRI刚好低于预警阈值"""
        cri = 0.49  # 刚好低于0.5
        risk_level = determine_risk_level(cri)
        
        assert risk_level == RiskLevel.SAFE, \
            f"CRI={cri}应判定为SAFE，实际为{risk_level}"
    
    def test_warning_level_at_lower_boundary(self):
        """测试预警等级：CRI刚好等于预警阈值"""
        cri = 0.5  # 等于预警阈值
        risk_level = determine_risk_level(cri)
        
        assert risk_level == RiskLevel.WARNING, \
            f"CRI={cri}应判定为WARNING，实际为{risk_level}"
    
    def test_warning_level_middle(self):
        """测试预警等级：CRI在预警和危险阈值之间"""
        cri = 0.6  # 在0.5和0.7之间
        risk_level = determine_risk_level(cri)
        
        assert risk_level == RiskLevel.WARNING, \
            f"CRI={cri}应判定为WARNING，实际为{risk_level}"
    
    def test_warning_level_at_upper_boundary(self):
        """测试预警等级：CRI刚好低于危险阈值"""
        cri = 0.69  # 刚好低于0.7
        risk_level = determine_risk_level(cri)
        
        assert risk_level == RiskLevel.WARNING, \
            f"CRI={cri}应判定为WARNING，实际为{risk_level}"
    
    def test_danger_level_at_boundary(self):
        """测试危险等级：CRI刚好等于危险阈值"""
        cri = 0.7  # 等于危险阈值
        risk_level = determine_risk_level(cri)
        
        assert risk_level == RiskLevel.DANGER, \
            f"CRI={cri}应判定为DANGER，实际为{risk_level}"
    
    def test_danger_level_high_cri(self):
        """测试危险等级：CRI远高于危险阈值"""
        cri = 0.9  # 远高于0.7
        risk_level = determine_risk_level(cri)
        
        assert risk_level == RiskLevel.DANGER, \
            f"CRI={cri}应判定为DANGER，实际为{risk_level}"
    
    def test_danger_level_maximum_cri(self):
        """测试危险等级：CRI为最大值1.0"""
        cri = 1.0  # 最大值
        risk_level = determine_risk_level(cri)
        
        assert risk_level == RiskLevel.DANGER, \
            f"CRI={cri}应判定为DANGER，实际为{risk_level}"
    
    def test_safe_level_minimum_cri(self):
        """测试安全等级：CRI为最小值0.0"""
        cri = 0.0  # 最小值
        risk_level = determine_risk_level(cri)
        
        assert risk_level == RiskLevel.SAFE, \
            f"CRI={cri}应判定为SAFE，实际为{risk_level}"
    
    def test_threshold_consistency(self):
        """测试阈值的一致性：确保阈值定义正确"""
        # 验证阈值常量的值
        assert RISK_THRESHOLD_WARNING == 0.5, \
            f"预警阈值应为0.5，实际为{RISK_THRESHOLD_WARNING}"
        assert RISK_THRESHOLD_DANGER == 0.7, \
            f"危险阈值应为0.7，实际为{RISK_THRESHOLD_DANGER}"
        
        # 验证危险阈值大于预警阈值
        assert RISK_THRESHOLD_DANGER > RISK_THRESHOLD_WARNING, \
            "危险阈值应大于预警阈值"
    
    def test_all_three_levels(self):
        """测试所有三个风险等级都能被正确判定"""
        # 测试一系列CRI值，确保覆盖所有三个等级
        test_cases = [
            (0.0, RiskLevel.SAFE),
            (0.3, RiskLevel.SAFE),
            (0.49, RiskLevel.SAFE),
            (0.5, RiskLevel.WARNING),
            (0.6, RiskLevel.WARNING),
            (0.69, RiskLevel.WARNING),
            (0.7, RiskLevel.DANGER),
            (0.85, RiskLevel.DANGER),
            (1.0, RiskLevel.DANGER),
        ]
        
        for cri, expected_level in test_cases:
            actual_level = determine_risk_level(cri)
            assert actual_level == expected_level, \
                f"CRI={cri}应判定为{expected_level}，实际为{actual_level}"


class TestRiskAssessmentIntegration:
    """风险评估集成测试
    
    测试 assess_collision_risk() 函数是否正确集成了风险等级判定
    Requirements: 4.1, 4.2, 4.3, 4.4
    """
    
    def test_high_risk_assessment(self):
        """测试高风险场景的完整评估"""
        # 高风险场景：近距离、短时间、正前方
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=90.0,
            sog=10.0
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=0.0,
            longitude=0.01,  # 约0.6海里
            heading=270.0,
            sog=12.0
        )
        
        risk = assess_collision_risk(own_ship, target_ship)
        
        # 验证CRI值较高
        assert risk.cri > 0.7, f"高风险场景CRI应大于0.7，实际为{risk.cri}"
        
        # 验证风险等级为DANGER
        assert risk.risk_level == RiskLevel.DANGER, \
            f"高风险场景应判定为DANGER，实际为{risk.risk_level}"
    
    def test_medium_risk_assessment(self):
        """测试中等风险场景的完整评估"""
        # 中等风险场景：中等距离、中等时间
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=90.0,
            sog=10.0
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=0.02,  # 北侧偏移，避免完全对遇
            longitude=0.08,  # 约4.8海里
            heading=270.0,
            sog=10.0
        )
        
        risk = assess_collision_risk(own_ship, target_ship)
        
        # 验证CRI值在预警范围
        assert 0.5 <= risk.cri < 0.7, \
            f"中等风险场景CRI应在0.5-0.7之间，实际为{risk.cri}"
        
        # 验证风险等级为WARNING
        assert risk.risk_level == RiskLevel.WARNING, \
            f"中等风险场景应判定为WARNING，实际为{risk.risk_level}"
    
    def test_low_risk_assessment(self):
        """测试低风险场景的完整评估"""
        # 低风险场景：远距离、长时间
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=90.0,
            sog=10.0
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=0.1,  # 北侧偏移，避免对遇
            longitude=0.3,  # 约18海里
            heading=270.0,
            sog=8.0
        )
        
        risk = assess_collision_risk(own_ship, target_ship)
        
        # 验证CRI值较低
        assert risk.cri < 0.5, f"低风险场景CRI应小于0.5，实际为{risk.cri}"
        
        # 验证风险等级为SAFE
        assert risk.risk_level == RiskLevel.SAFE, \
            f"低风险场景应判定为SAFE，实际为{risk.risk_level}"
    
    def test_risk_level_consistency_with_cri(self):
        """测试风险等级与CRI值的一致性"""
        # 创建多个场景，验证风险等级与CRI值的对应关系
        test_scenarios = [
            # (distance_deg, lat_offset, expected_min_cri, expected_max_cri, expected_level)
            (0.01, 0.0, 0.7, 1.0, RiskLevel.DANGER),      # 很近，对遇
            (0.08, 0.02, 0.5, 0.7, RiskLevel.WARNING),    # 中等，有偏移
            (0.3, 0.1, 0.0, 0.5, RiskLevel.SAFE),         # 较远，有偏移
        ]
        
        for distance_deg, lat_offset, min_cri, max_cri, expected_level in test_scenarios:
            own_ship = ShipState(
                mmsi=123456789,
                latitude=0.0,
                longitude=0.0,
                heading=90.0,
                sog=10.0
            )
            target_ship = ShipState(
                mmsi=987654321,
                latitude=lat_offset,
                longitude=distance_deg,
                heading=270.0,
                sog=10.0
            )
            
            risk = assess_collision_risk(own_ship, target_ship)
            
            # 验证CRI在预期范围内
            assert min_cri <= risk.cri <= max_cri, \
                f"距离{distance_deg}度(偏移{lat_offset})时，CRI应在{min_cri}-{max_cri}之间，实际为{risk.cri}"
            
            # 验证风险等级正确
            assert risk.risk_level == expected_level, \
                f"距离{distance_deg}度(偏移{lat_offset})时，风险等级应为{expected_level}，实际为{risk.risk_level}"


if __name__ == '__main__':
    pytest.main([__file__, '-v'])


# ============================================================================
# 风险阈值触发属性测试 (Property-Based Tests)
# ============================================================================

class TestRiskThresholdProperties:
    """风险阈值触发的属性测试
    
    使用 Hypothesis 进行基于属性的测试，验证风险阈值触发的一致性
    Requirements: 4.3, 4.4
    """
    
    @given(
        dcpa=st.floats(min_value=0.1, max_value=10.0, allow_nan=False, allow_infinity=False),
        tcpa=st.floats(min_value=1.0, max_value=60.0, allow_nan=False, allow_infinity=False),
        bearing=st.floats(min_value=-180.0, max_value=180.0, allow_nan=False, allow_infinity=False),
        speed_ratio=st.floats(min_value=0.1, max_value=3.0, allow_nan=False, allow_infinity=False)
    )
    def test_property_warning_threshold_consistency(self, dcpa, tcpa, bearing, speed_ratio):
        """
        Feature: maritime-collision-avoidance, Property 11: 风险阈值触发的一致性
        
        **Validates: Requirements 4.3, 4.4**
        
        属性：对于任意计算出的CRI值，当CRI超过预警阈值时，系统应发出预警信号
        
        这个属性验证了预警阈值触发的一致性：
        - 当 CRI >= RISK_THRESHOLD_WARNING (0.5) 时，风险等级应为 WARNING 或 DANGER
        - 当 CRI < RISK_THRESHOLD_WARNING (0.5) 时，风险等级应为 SAFE
        
        测试策略：
        1. 使用随机参数计算CRI值
        2. 根据CRI值判定风险等级
        3. 验证风险等级与CRI阈值的对应关系
        """
        # 计算CRI
        cri = calculate_cri(dcpa, tcpa, bearing, speed_ratio)
        
        # 判定风险等级
        risk_level = determine_risk_level(cri)
        
        # 验证预警阈值触发的一致性
        if cri >= RISK_THRESHOLD_WARNING:
            # CRI超过预警阈值，应触发预警或危险
            assert risk_level in [RiskLevel.WARNING, RiskLevel.DANGER], \
                f"CRI={cri:.4f} >= 预警阈值{RISK_THRESHOLD_WARNING}，应触发预警或危险，实际为{risk_level}"
        else:
            # CRI低于预警阈值，应为安全
            assert risk_level == RiskLevel.SAFE, \
                f"CRI={cri:.4f} < 预警阈值{RISK_THRESHOLD_WARNING}，应为安全，实际为{risk_level}"
    
    @given(
        dcpa=st.floats(min_value=0.1, max_value=10.0, allow_nan=False, allow_infinity=False),
        tcpa=st.floats(min_value=1.0, max_value=60.0, allow_nan=False, allow_infinity=False),
        bearing=st.floats(min_value=-180.0, max_value=180.0, allow_nan=False, allow_infinity=False),
        speed_ratio=st.floats(min_value=0.1, max_value=3.0, allow_nan=False, allow_infinity=False)
    )
    def test_property_danger_threshold_consistency(self, dcpa, tcpa, bearing, speed_ratio):
        """
        Feature: maritime-collision-avoidance, Property 11: 风险阈值触发的一致性
        
        **Validates: Requirements 4.3, 4.4**
        
        属性：对于任意计算出的CRI值，当CRI超过危险阈值时，系统应触发自动避让决策
        
        这个属性验证了危险阈值触发的一致性：
        - 当 CRI >= RISK_THRESHOLD_DANGER (0.7) 时，风险等级应为 DANGER
        - 当 CRI < RISK_THRESHOLD_DANGER (0.7) 时，风险等级应为 SAFE 或 WARNING
        
        测试策略：
        1. 使用随机参数计算CRI值
        2. 根据CRI值判定风险等级
        3. 验证风险等级与CRI阈值的对应关系
        """
        # 计算CRI
        cri = calculate_cri(dcpa, tcpa, bearing, speed_ratio)
        
        # 判定风险等级
        risk_level = determine_risk_level(cri)
        
        # 验证危险阈值触发的一致性
        if cri >= RISK_THRESHOLD_DANGER:
            # CRI超过危险阈值，应触发危险（自动避让）
            assert risk_level == RiskLevel.DANGER, \
                f"CRI={cri:.4f} >= 危险阈值{RISK_THRESHOLD_DANGER}，应触发危险（自动避让），实际为{risk_level}"
        else:
            # CRI低于危险阈值，应为安全或预警
            assert risk_level in [RiskLevel.SAFE, RiskLevel.WARNING], \
                f"CRI={cri:.4f} < 危险阈值{RISK_THRESHOLD_DANGER}，应为安全或预警，实际为{risk_level}"
    
    @given(
        dcpa=st.floats(min_value=0.1, max_value=10.0, allow_nan=False, allow_infinity=False),
        tcpa=st.floats(min_value=1.0, max_value=60.0, allow_nan=False, allow_infinity=False),
        bearing=st.floats(min_value=-180.0, max_value=180.0, allow_nan=False, allow_infinity=False),
        speed_ratio=st.floats(min_value=0.1, max_value=3.0, allow_nan=False, allow_infinity=False)
    )
    def test_property_threshold_ordering(self, dcpa, tcpa, bearing, speed_ratio):
        """
        Feature: maritime-collision-avoidance, Property 11: 风险阈值触发的一致性
        
        **Validates: Requirements 4.3, 4.4**
        
        属性：风险等级应遵循严格的顺序关系
        
        这个属性验证了风险等级的顺序一致性：
        - SAFE < WARNING < DANGER
        - 不应出现跳级或逆序的情况
        
        测试策略：
        1. 使用随机参数计算CRI值
        2. 根据CRI值判定风险等级
        3. 验证风险等级与CRI值的单调对应关系
        """
        # 计算CRI
        cri = calculate_cri(dcpa, tcpa, bearing, speed_ratio)
        
        # 判定风险等级
        risk_level = determine_risk_level(cri)
        
        # 验证风险等级与CRI值的对应关系
        if cri < RISK_THRESHOLD_WARNING:
            # CRI < 0.5，应为SAFE
            assert risk_level == RiskLevel.SAFE, \
                f"CRI={cri:.4f} < {RISK_THRESHOLD_WARNING}，应为SAFE，实际为{risk_level}"
        elif cri < RISK_THRESHOLD_DANGER:
            # 0.5 <= CRI < 0.7，应为WARNING
            assert risk_level == RiskLevel.WARNING, \
                f"{RISK_THRESHOLD_WARNING} <= CRI={cri:.4f} < {RISK_THRESHOLD_DANGER}，应为WARNING，实际为{risk_level}"
        else:
            # CRI >= 0.7，应为DANGER
            assert risk_level == RiskLevel.DANGER, \
                f"CRI={cri:.4f} >= {RISK_THRESHOLD_DANGER}，应为DANGER，实际为{risk_level}"
    
    @given(
        ship1_data=ship_state_strategy(
            lat_range=(-60.0, 60.0),
            lon_range=(-150.0, 150.0),
            speed_range=(1.0, 25.0)
        ),
        ship2_data=ship_state_strategy(
            lat_range=(-60.0, 60.0),
            lon_range=(-150.0, 150.0),
            speed_range=(1.0, 25.0)
        )
    )
    def test_property_integrated_threshold_consistency(self, ship1_data, ship2_data):
        """
        Feature: maritime-collision-avoidance, Property 11: 风险阈值触发的一致性
        
        **Validates: Requirements 4.3, 4.4**
        
        属性：在完整的风险评估流程中，风险阈值触发应保持一致性
        
        这个属性验证了从船舶状态到风险等级的完整流程：
        - assess_collision_risk() 返回的风险等级应与其CRI值一致
        - 风险等级应正确反映CRI阈值的触发情况
        
        测试策略：
        1. 使用随机船舶状态进行完整的风险评估
        2. 验证返回的风险等级与CRI值的一致性
        3. 确保阈值触发逻辑在集成场景中正确工作
        """
        # 过滤掉位置过于接近的船舶（避免数值不稳定）
        lat_diff = abs(ship1_data['latitude'] - ship2_data['latitude'])
        lon_diff = abs(ship1_data['longitude'] - ship2_data['longitude'])
        assume(lat_diff > 0.01 or lon_diff > 0.01)
        
        # 确保两艘船的 MMSI 不同
        if ship1_data['mmsi'] == ship2_data['mmsi']:
            if ship2_data['mmsi'] < 999999999:
                ship2_data['mmsi'] = ship2_data['mmsi'] + 1
            else:
                ship2_data['mmsi'] = 100000000
        
        # 创建船舶状态对象
        ship1 = ShipState(**ship1_data)
        ship2 = ShipState(**ship2_data)
        
        # 进行完整的风险评估
        risk = assess_collision_risk(ship1, ship2)
        
        # 验证风险等级与CRI值的一致性
        if risk.cri >= RISK_THRESHOLD_DANGER:
            assert risk.risk_level == RiskLevel.DANGER, \
                f"CRI={risk.cri:.4f} >= 危险阈值{RISK_THRESHOLD_DANGER}，风险等级应为DANGER，实际为{risk.risk_level}"
        elif risk.cri >= RISK_THRESHOLD_WARNING:
            assert risk.risk_level == RiskLevel.WARNING, \
                f"{RISK_THRESHOLD_WARNING} <= CRI={risk.cri:.4f} < {RISK_THRESHOLD_DANGER}，风险等级应为WARNING，实际为{risk.risk_level}"
        else:
            assert risk.risk_level == RiskLevel.SAFE, \
                f"CRI={risk.cri:.4f} < 预警阈值{RISK_THRESHOLD_WARNING}，风险等级应为SAFE，实际为{risk.risk_level}"
        
        # 验证CRI值在有效范围内
        assert 0.0 <= risk.cri <= 1.0, \
            f"CRI应在0-1范围内，实际为{risk.cri}"
    
    @given(
        cri_near_warning=st.floats(min_value=0.48, max_value=0.52, allow_nan=False, allow_infinity=False),
        cri_near_danger=st.floats(min_value=0.68, max_value=0.72, allow_nan=False, allow_infinity=False)
    )
    def test_property_threshold_boundary_precision(self, cri_near_warning, cri_near_danger):
        """
        Feature: maritime-collision-avoidance, Property 11: 风险阈值触发的一致性
        
        **Validates: Requirements 4.3, 4.4**
        
        属性：在阈值边界附近，风险等级判定应保持精确性
        
        这个属性验证了阈值边界的精确判定：
        - 在预警阈值（0.5）附近，判定应精确
        - 在危险阈值（0.7）附近，判定应精确
        - 不应出现边界模糊或判定错误
        
        测试策略：
        1. 生成接近阈值的CRI值
        2. 验证风险等级判定的精确性
        3. 确保边界情况处理正确
        """
        # 测试预警阈值边界
        risk_level_warning = determine_risk_level(cri_near_warning)
        
        if cri_near_warning >= RISK_THRESHOLD_WARNING:
            assert risk_level_warning in [RiskLevel.WARNING, RiskLevel.DANGER], \
                f"CRI={cri_near_warning:.4f} >= 预警阈值{RISK_THRESHOLD_WARNING}，应触发预警或危险，实际为{risk_level_warning}"
        else:
            assert risk_level_warning == RiskLevel.SAFE, \
                f"CRI={cri_near_warning:.4f} < 预警阈值{RISK_THRESHOLD_WARNING}，应为安全，实际为{risk_level_warning}"
        
        # 测试危险阈值边界
        risk_level_danger = determine_risk_level(cri_near_danger)
        
        if cri_near_danger >= RISK_THRESHOLD_DANGER:
            assert risk_level_danger == RiskLevel.DANGER, \
                f"CRI={cri_near_danger:.4f} >= 危险阈值{RISK_THRESHOLD_DANGER}，应触发危险，实际为{risk_level_danger}"
        else:
            assert risk_level_danger in [RiskLevel.SAFE, RiskLevel.WARNING], \
                f"CRI={cri_near_danger:.4f} < 危险阈值{RISK_THRESHOLD_DANGER}，应为安全或预警，实际为{risk_level_danger}"
    
    @given(
        dcpa=st.floats(min_value=0.1, max_value=10.0, allow_nan=False, allow_infinity=False),
        tcpa=st.floats(min_value=1.0, max_value=60.0, allow_nan=False, allow_infinity=False),
        bearing=st.floats(min_value=-180.0, max_value=180.0, allow_nan=False, allow_infinity=False),
        speed_ratio=st.floats(min_value=0.1, max_value=3.0, allow_nan=False, allow_infinity=False)
    )
    def test_property_no_threshold_skip(self, dcpa, tcpa, bearing, speed_ratio):
        """
        Feature: maritime-collision-avoidance, Property 11: 风险阈值触发的一致性
        
        **Validates: Requirements 4.3, 4.4**
        
        属性：风险等级不应跳过中间级别
        
        这个属性验证了风险等级的连续性：
        - 不应从SAFE直接跳到DANGER（应经过WARNING）
        - 风险等级应遵循 SAFE -> WARNING -> DANGER 的顺序
        
        测试策略：
        1. 计算CRI值并判定风险等级
        2. 验证风险等级与CRI值的对应关系
        3. 确保不存在跳级现象
        """
        # 计算CRI
        cri = calculate_cri(dcpa, tcpa, bearing, speed_ratio)
        
        # 判定风险等级
        risk_level = determine_risk_level(cri)
        
        # 验证风险等级的连续性
        if risk_level == RiskLevel.SAFE:
            # SAFE级别，CRI应小于预警阈值
            assert cri < RISK_THRESHOLD_WARNING, \
                f"风险等级为SAFE时，CRI={cri:.4f}应小于预警阈值{RISK_THRESHOLD_WARNING}"
        elif risk_level == RiskLevel.WARNING:
            # WARNING级别，CRI应在预警和危险阈值之间
            assert RISK_THRESHOLD_WARNING <= cri < RISK_THRESHOLD_DANGER, \
                f"风险等级为WARNING时，CRI={cri:.4f}应在{RISK_THRESHOLD_WARNING}和{RISK_THRESHOLD_DANGER}之间"
        elif risk_level == RiskLevel.DANGER:
            # DANGER级别，CRI应大于等于危险阈值
            assert cri >= RISK_THRESHOLD_DANGER, \
                f"风险等级为DANGER时，CRI={cri:.4f}应大于等于危险阈值{RISK_THRESHOLD_DANGER}"
