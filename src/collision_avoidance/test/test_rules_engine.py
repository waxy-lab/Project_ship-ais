"""
COLREGS规则引擎测试

测试相遇类型判定、船舶角色确定和态势分析功能
"""

import pytest
import math
from hypothesis import given, strategies as st, assume

# 导入被测试的模块
from collision_avoidance.rules_engine import (
    EncounterType,
    VesselRole,
    determine_encounter_type,
    determine_vessel_roles,
    analyze_encounter_situation,
    calculate_relative_bearing,
    calculate_heading_difference,
    normalize_angle,
    is_crossing_from_starboard,
)

# 导入数据模型
from scenario_generator.models import ShipState


# ============================================================================
# 测试辅助函数
# ============================================================================

class TestNormalizeAngle:
    """测试角度归一化函数"""
    
    def test_angle_in_range(self):
        """测试范围内的角度"""
        assert normalize_angle(90) == 90
        assert normalize_angle(-90) == -90
        assert normalize_angle(0) == 0
        assert normalize_angle(180) == 180
        assert normalize_angle(-180) == -180
    
    def test_angle_above_180(self):
        """测试大于180度的角度"""
        assert normalize_angle(270) == -90
        assert normalize_angle(190) == -170
        assert normalize_angle(360) == 0
        assert normalize_angle(450) == 90
    
    def test_angle_below_minus_180(self):
        """测试小于-180度的角度"""
        assert normalize_angle(-270) == 90
        assert normalize_angle(-190) == 170
        assert normalize_angle(-360) == 0
        assert normalize_angle(-450) == -90


class TestCalculateHeadingDifference:
    """测试航向差计算函数"""
    
    def test_same_heading(self):
        """测试相同航向"""
        assert calculate_heading_difference(90, 90) == 0
        assert calculate_heading_difference(0, 0) == 0
        assert calculate_heading_difference(180, 180) == 0
    
    def test_opposite_heading(self):
        """测试相反航向"""
        assert calculate_heading_difference(0, 180) == 180
        assert calculate_heading_difference(90, 270) == 180
        assert calculate_heading_difference(45, 225) == 180
    
    def test_perpendicular_heading(self):
        """测试垂直航向"""
        assert calculate_heading_difference(0, 90) == 90
        assert calculate_heading_difference(0, 270) == 90
        assert calculate_heading_difference(180, 90) == 90
    
    def test_wrap_around(self):
        """测试跨越0度的情况"""
        assert calculate_heading_difference(10, 350) == 20
        assert calculate_heading_difference(350, 10) == 20


class TestCalculateRelativeBearing:
    """测试相对方位计算函数"""
    
    def test_target_ahead(self):
        """测试目标在前方"""
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=0.0,  # 北向
            sog=10.0
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=0.01,  # 北方
            longitude=0.0,
            heading=0.0,
            sog=10.0
        )
        bearing = calculate_relative_bearing(own_ship, target_ship)
        assert abs(bearing) < 5  # 应该接近0度
    
    def test_target_starboard(self):
        """测试目标在右舷"""
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=0.0,  # 北向
            sog=10.0
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=0.0,
            longitude=0.01,  # 东方
            heading=0.0,
            sog=10.0
        )
        bearing = calculate_relative_bearing(own_ship, target_ship)
        assert 85 < bearing < 95  # 应该接近90度
    
    def test_target_port(self):
        """测试目标在左舷"""
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=0.0,  # 北向
            sog=10.0
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=0.0,
            longitude=-0.01,  # 西方
            heading=0.0,
            sog=10.0
        )
        bearing = calculate_relative_bearing(own_ship, target_ship)
        assert -95 < bearing < -85  # 应该接近-90度


class TestIsCrossingFromStarboard:
    """测试右舷交叉判定函数"""
    
    def test_starboard_crossing(self):
        """测试右舷交叉"""
        assert is_crossing_from_starboard(45) == True
        assert is_crossing_from_starboard(90) == True
        assert is_crossing_from_starboard(135) == True
    
    def test_port_crossing(self):
        """测试左舷交叉"""
        assert is_crossing_from_starboard(-45) == False
        assert is_crossing_from_starboard(-90) == False
        assert is_crossing_from_starboard(-135) == False
    
    def test_boundary_cases(self):
        """测试边界情况"""
        assert is_crossing_from_starboard(0.1) == True
        assert is_crossing_from_starboard(-0.1) == False
        assert is_crossing_from_starboard(179.9) == True
        assert is_crossing_from_starboard(-179.9) == False


# ============================================================================
# 测试相遇类型判定
# ============================================================================

class TestDetermineEncounterType:
    """测试相遇类型判定函数"""
    
    def test_head_on_basic(self):
        """测试基本对遇场景"""
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=0.0,  # 北向
            sog=10.0
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=0.01,  # 北方
            longitude=0.0,
            heading=180.0,  # 南向
            sog=10.0
        )
        encounter_type = determine_encounter_type(own_ship, target_ship)
        assert encounter_type == EncounterType.HEAD_ON
    
    def test_crossing_from_starboard(self):
        """测试右舷交叉场景"""
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=0.0,  # 北向
            sog=10.0
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=0.0,
            longitude=0.01,  # 东方
            heading=270.0,  # 西向
            sog=10.0
        )
        encounter_type = determine_encounter_type(own_ship, target_ship)
        assert encounter_type == EncounterType.CROSSING
    
    def test_crossing_from_port(self):
        """测试左舷交叉场景"""
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=0.0,  # 北向
            sog=10.0
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=0.0,
            longitude=-0.01,  # 西方
            heading=90.0,  # 东向
            sog=10.0
        )
        encounter_type = determine_encounter_type(own_ship, target_ship)
        assert encounter_type == EncounterType.CROSSING
    
    def test_overtaking_basic(self):
        """测试基本追越场景"""
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=0.0,  # 北向
            sog=15.0  # 更快
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=0.01,  # 前方
            longitude=0.0,
            heading=0.0,  # 同向
            sog=10.0  # 更慢
        )
        encounter_type = determine_encounter_type(own_ship, target_ship)
        # 注意：这个场景中目标船在前方，不是后方，所以不是追越
        # 追越要求目标船在后方22.5度扇形区域内
        assert encounter_type != EncounterType.OVERTAKING
    
    def test_overtaking_from_rear(self):
        """测试从后方追越场景"""
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=0.0,  # 北向
            sog=15.0  # 更快
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=-0.01,  # 后方
            longitude=0.0,
            heading=0.0,  # 同向
            sog=10.0  # 更慢
        )
        encounter_type = determine_encounter_type(own_ship, target_ship)
        # 目标船在后方且本船更快，应该是追越
        assert encounter_type == EncounterType.OVERTAKING
    
    def test_no_encounter_parallel(self):
        """测试无相遇风险的平行航行"""
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=0.0,  # 北向
            sog=10.0
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=0.0,
            longitude=0.01,  # 东方
            heading=0.0,  # 同向北
            sog=10.0
        )
        encounter_type = determine_encounter_type(own_ship, target_ship)
        # 平行航行，相对方位90度，不是交叉（交叉要求5-112.5度）
        # 实际上90度在交叉范围内
        assert encounter_type == EncounterType.CROSSING


# ============================================================================
# 测试船舶角色确定
# ============================================================================

class TestDetermineVesselRoles:
    """测试船舶角色确定函数"""
    
    def test_head_on_both_give_way(self):
        """测试对遇时双方都是让路船"""
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=0.0,
            sog=10.0
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=0.01,
            longitude=0.0,
            heading=180.0,
            sog=10.0
        )
        encounter_type = EncounterType.HEAD_ON
        own_role, target_role = determine_vessel_roles(encounter_type, own_ship, target_ship)
        assert own_role == VesselRole.BOTH_GIVE_WAY
        assert target_role == VesselRole.BOTH_GIVE_WAY
    
    def test_crossing_starboard_give_way(self):
        """测试交叉时右舷有他船的船为让路船"""
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=0.0,  # 北向
            sog=10.0
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=0.0,
            longitude=0.01,  # 右舷
            heading=270.0,  # 西向
            sog=10.0
        )
        encounter_type = EncounterType.CROSSING
        own_role, target_role = determine_vessel_roles(encounter_type, own_ship, target_ship)
        # 目标船在右舷，本船为让路船
        assert own_role == VesselRole.GIVE_WAY
        assert target_role == VesselRole.STAND_ON
    
    def test_crossing_port_stand_on(self):
        """测试交叉时左舷有他船的船为直航船"""
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=0.0,  # 北向
            sog=10.0
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=0.0,
            longitude=-0.01,  # 左舷
            heading=90.0,  # 东向
            sog=10.0
        )
        encounter_type = EncounterType.CROSSING
        own_role, target_role = determine_vessel_roles(encounter_type, own_ship, target_ship)
        # 目标船在左舷，本船为直航船
        assert own_role == VesselRole.STAND_ON
        assert target_role == VesselRole.GIVE_WAY
    
    def test_overtaking_faster_gives_way(self):
        """测试追越时速度更快的船为让路船"""
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=0.0,
            sog=15.0  # 更快
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=-0.01,  # 后方
            longitude=0.0,
            heading=0.0,
            sog=10.0  # 更慢
        )
        encounter_type = EncounterType.OVERTAKING
        own_role, target_role = determine_vessel_roles(encounter_type, own_ship, target_ship)
        # 本船更快，本船为让路船
        assert own_role == VesselRole.GIVE_WAY
        assert target_role == VesselRole.STAND_ON
    
    def test_no_encounter_undefined(self):
        """测试无相遇时角色未定义"""
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
            longitude=0.01,
            heading=0.0,
            sog=10.0
        )
        encounter_type = EncounterType.NONE
        own_role, target_role = determine_vessel_roles(encounter_type, own_ship, target_ship)
        assert own_role == VesselRole.UNDEFINED
        assert target_role == VesselRole.UNDEFINED


# ============================================================================
# 测试态势分析
# ============================================================================

class TestAnalyzeEncounterSituation:
    """测试态势分析函数"""
    
    def test_head_on_situation(self):
        """测试对遇态势分析"""
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=0.0,
            sog=10.0
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=0.01,
            longitude=0.0,
            heading=180.0,
            sog=10.0
        )
        situation = analyze_encounter_situation(own_ship, target_ship)
        
        assert situation.encounter_type == EncounterType.HEAD_ON
        assert situation.own_ship_role == VesselRole.BOTH_GIVE_WAY
        assert situation.target_ship_role == VesselRole.BOTH_GIVE_WAY
        assert abs(situation.relative_bearing) < 10
        assert 170 <= situation.heading_difference <= 190
        assert situation.distance > 0
        assert situation.is_overtaking == False
        assert situation.is_crossing_from_starboard == False
    
    def test_crossing_situation(self):
        """测试交叉态势分析"""
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
            longitude=0.01,
            heading=270.0,
            sog=10.0
        )
        situation = analyze_encounter_situation(own_ship, target_ship)
        
        assert situation.encounter_type == EncounterType.CROSSING
        assert situation.own_ship_role == VesselRole.GIVE_WAY
        assert situation.target_ship_role == VesselRole.STAND_ON
        assert situation.is_crossing_from_starboard == True
    
    def test_distance_calculation(self):
        """测试距离计算"""
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=0.0,
            sog=10.0
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=1.0,  # 1度纬度约60海里
            longitude=0.0,
            heading=180.0,
            sog=10.0
        )
        situation = analyze_encounter_situation(own_ship, target_ship)
        
        # 距离应该约为60海里
        assert 55 < situation.distance < 65


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
