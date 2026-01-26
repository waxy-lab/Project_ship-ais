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
from collision_avoidance.risk_assessment import calculate_dcpa_tcpa
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
