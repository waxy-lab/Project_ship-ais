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
    TurnDirection,
    ActionType,
    AvoidanceAction,
    determine_encounter_type,
    determine_vessel_roles,
    analyze_encounter_situation,
    calculate_relative_bearing,
    calculate_heading_difference,
    normalize_angle,
    is_crossing_from_starboard,
    apply_colregs_rule,
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
    
    def test_head_on_boundary_170_degrees(self):
        """测试对遇边界情况：航向差170度"""
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
            heading=170.0,  # 170度（边界值）
            sog=10.0
        )
        encounter_type = determine_encounter_type(own_ship, target_ship)
        assert encounter_type == EncounterType.HEAD_ON
    
    def test_head_on_boundary_190_degrees(self):
        """测试对遇边界情况：航向差190度"""
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
            heading=190.0,  # 190度（边界值）
            sog=10.0
        )
        encounter_type = determine_encounter_type(own_ship, target_ship)
        assert encounter_type == EncounterType.HEAD_ON
    
    def test_head_on_boundary_relative_bearing_10(self):
        """测试对遇边界情况：相对方位10度"""
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=0.0,  # 北向
            sog=10.0
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=0.01,  # 北方偏东
            longitude=0.0017,  # 约10度偏角
            heading=180.0,  # 南向
            sog=10.0
        )
        encounter_type = determine_encounter_type(own_ship, target_ship)
        assert encounter_type == EncounterType.HEAD_ON
    
    def test_head_on_boundary_relative_bearing_minus_10(self):
        """测试对遇边界情况：相对方位-10度"""
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=0.0,  # 北向
            sog=10.0
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=0.01,  # 北方偏西
            longitude=-0.0017,  # 约-10度偏角
            heading=180.0,  # 南向
            sog=10.0
        )
        encounter_type = determine_encounter_type(own_ship, target_ship)
        assert encounter_type == EncounterType.HEAD_ON
    
    def test_not_head_on_heading_diff_169(self):
        """测试非对遇：航向差169度（刚好不满足对遇条件）"""
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
            heading=169.0,  # 169度（不满足对遇）
            sog=10.0
        )
        encounter_type = determine_encounter_type(own_ship, target_ship)
        assert encounter_type != EncounterType.HEAD_ON
    
    def test_not_head_on_heading_diff_191(self):
        """测试非对遇：航向差191度（刚好不满足对遇条件）"""
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
            heading=191.0,  # 191度（不满足对遇）
            sog=10.0
        )
        encounter_type = determine_encounter_type(own_ship, target_ship)
        assert encounter_type != EncounterType.HEAD_ON
    
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
    
    def test_crossing_boundary_5_degrees(self):
        """测试交叉边界情况：相对方位5度"""
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=0.0,  # 北向
            sog=10.0
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=0.01,  # 北方偏东
            longitude=0.0009,  # 约5度偏角
            heading=270.0,  # 西向
            sog=10.0
        )
        encounter_type = determine_encounter_type(own_ship, target_ship)
        assert encounter_type == EncounterType.CROSSING
    
    def test_crossing_boundary_112_5_degrees(self):
        """测试交叉边界情况：相对方位112.5度"""
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=0.0,  # 北向
            sog=10.0
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=0.004,  # 东南方
            longitude=0.01,  # 约112.5度
            heading=270.0,  # 西向
            sog=10.0
        )
        encounter_type = determine_encounter_type(own_ship, target_ship)
        assert encounter_type == EncounterType.CROSSING
    
    def test_crossing_45_degrees(self):
        """测试交叉场景：相对方位45度"""
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=0.0,  # 北向
            sog=10.0
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=0.007,  # 东北方
            longitude=0.007,  # 约45度
            heading=225.0,  # 西南向
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
    
    def test_overtaking_boundary_112_5_degrees_starboard(self):
        """测试追越边界情况：相对方位112.5度（右舷后方）"""
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=0.0,  # 北向
            sog=15.0  # 更快
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=-0.004,  # 后方偏右
            longitude=0.01,  # 约112.5度
            heading=0.0,  # 同向
            sog=10.0  # 更慢
        )
        encounter_type = determine_encounter_type(own_ship, target_ship)
        # 相对方位刚好112.5度，不应该是追越（追越要求>112.5度）
        assert encounter_type != EncounterType.OVERTAKING
    
    def test_overtaking_boundary_113_degrees_starboard(self):
        """测试追越边界情况：相对方位113度（刚好满足追越）"""
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
            longitude=0.0025,  # 约113度（后方偏右）
            heading=0.0,  # 同向
            sog=10.0  # 更慢
        )
        encounter_type = determine_encounter_type(own_ship, target_ship)
        # 相对方位约113度，应该是追越（>112.5度且在后方）
        assert encounter_type == EncounterType.OVERTAKING
    
    def test_overtaking_boundary_minus_112_5_degrees_port(self):
        """测试追越边界情况：相对方位-112.5度（左舷后方）"""
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=0.0,  # 北向
            sog=15.0  # 更快
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=-0.004,  # 后方偏左
            longitude=-0.01,  # 约-112.5度
            heading=0.0,  # 同向
            sog=10.0  # 更慢
        )
        encounter_type = determine_encounter_type(own_ship, target_ship)
        # 相对方位刚好-112.5度，不应该是追越
        assert encounter_type != EncounterType.OVERTAKING
    
    def test_overtaking_180_degrees(self):
        """测试追越场景：相对方位180度（正后方）"""
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=0.0,  # 北向
            sog=15.0  # 更快
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=-0.01,  # 正后方
            longitude=0.0,
            heading=0.0,  # 同向
            sog=10.0  # 更慢
        )
        encounter_type = determine_encounter_type(own_ship, target_ship)
        assert encounter_type == EncounterType.OVERTAKING
    
    def test_overtaking_requires_higher_speed(self):
        """测试追越要求本船速度更快"""
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=0.0,  # 北向
            sog=10.0  # 相同速度
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=-0.01,  # 后方
            longitude=0.0,
            heading=0.0,  # 同向
            sog=10.0  # 相同速度
        )
        encounter_type = determine_encounter_type(own_ship, target_ship)
        # 速度相同，不是追越
        assert encounter_type != EncounterType.OVERTAKING
    
    def test_overtaking_slower_ship_in_rear(self):
        """测试后方船速度更慢不是追越"""
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=0.0,  # 北向
            sog=10.0  # 更慢
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=-0.01,  # 后方
            longitude=0.0,
            heading=0.0,  # 同向
            sog=15.0  # 更快
        )
        encounter_type = determine_encounter_type(own_ship, target_ship)
        # 本船更慢，不是追越
        assert encounter_type != EncounterType.OVERTAKING
    
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
    
    def test_no_encounter_diverging(self):
        """测试无相遇风险的背离航行"""
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=0.0,  # 北向
            sog=10.0
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=-0.01,  # 南方
            longitude=0.0,
            heading=0.0,  # 北向（背离）
            sog=10.0
        )
        encounter_type = determine_encounter_type(own_ship, target_ship)
        # 背离航行，相对方位180度，不满足任何相遇类型
        assert encounter_type == EncounterType.NONE
    
    def test_no_encounter_relative_bearing_4_degrees(self):
        """测试无相遇：相对方位4度（不满足交叉条件）"""
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=0.0,  # 北向
            sog=10.0
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=0.01,  # 北方偏东
            longitude=0.0007,  # 约4度偏角
            heading=270.0,  # 西向
            sog=10.0
        )
        encounter_type = determine_encounter_type(own_ship, target_ship)
        # 相对方位4度，不满足交叉条件（要求>=5度）
        assert encounter_type == EncounterType.NONE
    
    def test_encounter_type_with_different_headings(self):
        """测试不同航向组合的相遇类型"""
        # 东向船和西向船对遇
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=90.0,  # 东向
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
        assert encounter_type == EncounterType.HEAD_ON
    
    def test_encounter_type_wrap_around_360(self):
        """测试航向跨越360度的情况"""
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=350.0,  # 接近北向
            sog=10.0
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=0.01,  # 北方
            longitude=0.0,
            heading=170.0,  # 接近南向
            sog=10.0
        )
        encounter_type = determine_encounter_type(own_ship, target_ship)
        assert encounter_type == EncounterType.HEAD_ON


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


# ============================================================================
# 测试COLREGS规则应用
# ============================================================================

class TestApplyCOLREGSRule:
    """测试COLREGS规则应用函数
    
    Requirements: 3.1-3.6
    """
    
    def test_head_on_turn_starboard(self):
        """测试对遇时向右转向
        
        Requirements: 3.1
        """
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
        
        action = apply_colregs_rule(EncounterType.HEAD_ON, own_ship, target_ship)
        
        assert action.action_type == ActionType.COURSE_CHANGE
        assert action.turn_direction == TurnDirection.STARBOARD
        assert action.turn_angle == 15.0
        assert "Rule 14" in action.reason
    
    def test_crossing_give_way_turn_starboard(self):
        """测试交叉相遇时让路船向右转向
        
        Requirements: 3.2
        """
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
        
        action = apply_colregs_rule(EncounterType.CROSSING, own_ship, target_ship)
        
        assert action.action_type == ActionType.COURSE_CHANGE
        assert action.turn_direction == TurnDirection.STARBOARD
        assert action.turn_angle == 30.0  # 明显避让
        assert "Rule 15" in action.reason
        assert "让路船" in action.reason
    
    def test_crossing_stand_on_maintain_course(self):
        """测试交叉相遇时直航船保持航向
        
        Requirements: 3.3
        """
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
        
        action = apply_colregs_rule(EncounterType.CROSSING, own_ship, target_ship)
        
        assert action.action_type == ActionType.MAINTAIN
        assert action.maintain_course == True
        assert "Rule 17" in action.reason
        assert "直航船" in action.reason
    
    def test_overtaking_give_way_avoidance(self):
        """测试追越时追越船避让
        
        Requirements: 3.4
        """
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
        
        action = apply_colregs_rule(EncounterType.OVERTAKING, own_ship, target_ship)
        
        assert action.action_type == ActionType.COURSE_CHANGE
        assert action.turn_direction in [TurnDirection.STARBOARD, TurnDirection.PORT]
        assert action.turn_angle == 20.0
        assert "Rule 13" in action.reason
        assert "追越船" in action.reason
    
    def test_overtaking_stand_on_maintain_course(self):
        """测试追越时被追越船保持航向
        
        Requirements: 3.4
        """
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=0.0,  # 北向
            sog=10.0  # 更慢
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=-0.01,  # 后方
            longitude=0.0,
            heading=0.0,  # 同向
            sog=15.0  # 更快
        )
        
        action = apply_colregs_rule(EncounterType.OVERTAKING, own_ship, target_ship)
        
        assert action.action_type == ActionType.MAINTAIN
        assert action.maintain_course == True
        assert "Rule 13" in action.reason
        assert "被追越船" in action.reason
    
    def test_overtaking_turn_direction_based_on_bearing(self):
        """测试追越时根据相对方位选择转向方向"""
        # 目标船在右后方，应该向右转
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
            longitude=0.005,  # 右后方
            heading=0.0,  # 同向
            sog=10.0  # 更慢
        )
        
        action = apply_colregs_rule(EncounterType.OVERTAKING, own_ship, target_ship)
        
        assert action.turn_direction == TurnDirection.STARBOARD
        
        # 目标船在左后方，应该向左转
        target_ship2 = ShipState(
            mmsi=987654321,
            latitude=-0.01,  # 后方
            longitude=-0.005,  # 左后方
            heading=0.0,  # 同向
            sog=10.0  # 更慢
        )
        
        action2 = apply_colregs_rule(EncounterType.OVERTAKING, own_ship, target_ship2)
        
        assert action2.turn_direction == TurnDirection.PORT
    
    def test_no_encounter_no_action(self):
        """测试无相遇风险时无需动作"""
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=0.0,  # 北向
            sog=10.0
        )
        target_ship = ShipState(
            mmsi=987654321,
            latitude=-0.01,  # 南方
            longitude=0.0,
            heading=0.0,  # 北向（背离）
            sog=10.0
        )
        
        action = apply_colregs_rule(EncounterType.NONE, own_ship, target_ship)
        
        assert action.action_type == ActionType.NO_ACTION
        assert action.no_action == True
        assert "无相遇风险" in action.reason
    
    def test_action_validation(self):
        """测试避让动作的数据验证"""
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
        
        action = apply_colregs_rule(EncounterType.HEAD_ON, own_ship, target_ship)
        
        # 验证动作包含必要的信息
        assert action.turn_direction is not None
        assert action.turn_angle is not None
        assert action.turn_angle > 0
        assert action.reason != ""
    
    def test_different_encounter_scenarios(self):
        """测试不同相遇场景的规则应用"""
        own_ship = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=90.0,  # 东向
            sog=10.0
        )
        
        # 场景1：对遇
        target_ship_head_on = ShipState(
            mmsi=987654321,
            latitude=0.0,
            longitude=0.01,  # 东方
            heading=270.0,  # 西向
            sog=10.0
        )
        action1 = apply_colregs_rule(EncounterType.HEAD_ON, own_ship, target_ship_head_on)
        assert action1.action_type == ActionType.COURSE_CHANGE
        assert action1.turn_direction == TurnDirection.STARBOARD
        
        # 场景2：交叉（让路船）- 目标船在右舷
        own_ship2 = ShipState(
            mmsi=123456789,
            latitude=0.0,
            longitude=0.0,
            heading=0.0,  # 北向
            sog=10.0
        )
        target_ship_crossing = ShipState(
            mmsi=987654321,
            latitude=0.0,
            longitude=0.01,  # 右舷（东方）
            heading=270.0,  # 西向
            sog=10.0
        )
        action2 = apply_colregs_rule(EncounterType.CROSSING, own_ship2, target_ship_crossing)
        assert action2.action_type == ActionType.COURSE_CHANGE
        assert action2.turn_angle == 30.0
        
        # 场景3：追越
        target_ship_overtaking = ShipState(
            mmsi=987654321,
            latitude=0.0,
            longitude=-0.01,  # 西方（后方）
            heading=90.0,  # 东向
            sog=8.0  # 更慢
        )
        action3 = apply_colregs_rule(EncounterType.OVERTAKING, own_ship, target_ship_overtaking)
        assert action3.action_type == ActionType.COURSE_CHANGE
        assert action3.turn_angle == 20.0


# ============================================================================
# 属性测试（Property-Based Tests）
# ============================================================================

class TestPropertyHeadOnTurnDirection:
    """
    Property 7: 对遇规则的转向方向
    
    Feature: maritime-collision-avoidance, Property 7: 对遇规则的转向方向
    Validates: Requirements 3.1
    
    对于任何检测到的对遇局面，避碰决策模块输出的转向指令应为向右（starboard）转向
    """
    
    @given(
        own_lat=st.floats(min_value=-80.0, max_value=80.0, allow_nan=False, allow_infinity=False),
        own_lon=st.floats(min_value=-170.0, max_value=170.0, allow_nan=False, allow_infinity=False),
        own_heading=st.floats(min_value=0.0, max_value=359.999, allow_nan=False, allow_infinity=False),
        own_speed=st.floats(min_value=5.0, max_value=25.0, allow_nan=False, allow_infinity=False),
        distance=st.floats(min_value=0.5, max_value=10.0, allow_nan=False, allow_infinity=False),
        target_speed=st.floats(min_value=5.0, max_value=25.0, allow_nan=False, allow_infinity=False),
        heading_offset=st.floats(min_value=-10.0, max_value=10.0, allow_nan=False, allow_infinity=False),
        relative_bearing_offset=st.floats(min_value=-5.0, max_value=5.0, allow_nan=False, allow_infinity=False)
    )
    def test_property_head_on_always_turns_starboard(
        self, 
        own_lat, 
        own_lon, 
        own_heading, 
        own_speed,
        distance,
        target_speed,
        heading_offset,
        relative_bearing_offset
    ):
        """
        属性测试：对于任何对遇场景，系统应该输出向右转向指令
        
        测试策略：
        1. 生成随机的本船状态（位置、航向、速度）
        2. 根据对遇条件生成目标船状态：
           - 航向差在170-190度之间（相向航行）
           - 相对方位在-10到10度之间（正前方）
        3. 验证apply_colregs_rule返回的动作为向右转向
        
        Requirements: 3.1
        """
        # 创建本船状态
        own_ship = ShipState(
            mmsi=123456789,
            latitude=own_lat,
            longitude=own_lon,
            heading=own_heading,
            sog=own_speed
        )
        
        # 计算目标船航向（相向航行，航向差约180度）
        target_heading = (own_heading + 180.0 + heading_offset) % 360.0
        
        # 计算目标船位置（在本船前方，距离为distance海里）
        # 相对方位接近0度（正前方），允许小偏差
        bearing_to_target = own_heading + relative_bearing_offset
        
        # 将距离从海里转换为度（1度约60海里）
        distance_degrees = distance / 60.0
        
        # 计算目标船的纬度和经度
        bearing_rad = math.radians(bearing_to_target)
        target_lat = own_lat + distance_degrees * math.cos(bearing_rad)
        target_lon = own_lon + distance_degrees * math.sin(bearing_rad) / math.cos(math.radians(own_lat))
        
        # 确保目标船位置在有效范围内
        assume(-90.0 <= target_lat <= 90.0)
        assume(-180.0 <= target_lon <= 180.0)
        
        # 创建目标船状态
        target_ship = ShipState(
            mmsi=987654321,
            latitude=target_lat,
            longitude=target_lon,
            heading=target_heading,
            sog=target_speed
        )
        
        # 验证这确实是对遇场景
        encounter_type = determine_encounter_type(own_ship, target_ship)
        
        # 只测试对遇场景
        assume(encounter_type == EncounterType.HEAD_ON)
        
        # 应用COLREGS规则
        action = apply_colregs_rule(encounter_type, own_ship, target_ship)
        
        # 验证属性：对遇时必须向右转向
        assert action.action_type == ActionType.COURSE_CHANGE, \
            f"对遇场景应该返回航向改变动作，但得到: {action.action_type}"
        
        assert action.turn_direction == TurnDirection.STARBOARD, \
            f"对遇场景应该向右转向，但得到: {action.turn_direction}"
        
        assert action.turn_angle is not None and action.turn_angle > 0, \
            f"转向角度应该大于0，但得到: {action.turn_angle}"
        
        assert "Rule 14" in action.reason, \
            f"动作理由应该包含'Rule 14'，但得到: {action.reason}"


class TestPropertyGiveWayObviousAvoidance:
    """
    Property 8: 让路船的明显避让
    
    Feature: maritime-collision-avoidance, Property 8: 让路船的明显避让
    Validates: Requirements 3.2
    
    对于任何交叉相遇场景中被判定为让路船的情况，避碰决策模块输出的航向改变应大于等于15度
    """
    
    @given(
        own_lat=st.floats(min_value=-80.0, max_value=80.0, allow_nan=False, allow_infinity=False),
        own_lon=st.floats(min_value=-170.0, max_value=170.0, allow_nan=False, allow_infinity=False),
        own_heading=st.floats(min_value=0.0, max_value=359.999, allow_nan=False, allow_infinity=False),
        own_speed=st.floats(min_value=5.0, max_value=25.0, allow_nan=False, allow_infinity=False),
        distance=st.floats(min_value=0.5, max_value=10.0, allow_nan=False, allow_infinity=False),
        target_speed=st.floats(min_value=5.0, max_value=25.0, allow_nan=False, allow_infinity=False),
        relative_bearing=st.floats(min_value=5.0, max_value=112.5, allow_nan=False, allow_infinity=False),
        target_heading_offset=st.floats(min_value=-90.0, max_value=90.0, allow_nan=False, allow_infinity=False)
    )
    def test_property_give_way_obvious_avoidance(
        self,
        own_lat,
        own_lon,
        own_heading,
        own_speed,
        distance,
        target_speed,
        relative_bearing,
        target_heading_offset
    ):
        """
        属性测试：对于任何交叉相遇场景中的让路船，航向改变应大于等于15度
        
        测试策略：
        1. 生成随机的本船状态（位置、航向、速度）
        2. 根据交叉相遇条件生成目标船状态：
           - 目标船在右舷（相对方位5-112.5度）
           - 这样本船就是让路船
        3. 验证apply_colregs_rule返回的航向改变角度 >= 15度
        
        Requirements: 3.2
        """
        # 创建本船状态
        own_ship = ShipState(
            mmsi=123456789,
            latitude=own_lat,
            longitude=own_lon,
            heading=own_heading,
            sog=own_speed
        )
        
        # 计算目标船位置（在本船右舷，相对方位在5-112.5度之间）
        bearing_to_target = own_heading + relative_bearing
        
        # 将距离从海里转换为度（1度约60海里）
        distance_degrees = distance / 60.0
        
        # 计算目标船的纬度和经度
        bearing_rad = math.radians(bearing_to_target)
        target_lat = own_lat + distance_degrees * math.cos(bearing_rad)
        
        # 考虑纬度修正
        if abs(own_lat) < 89.0:  # 避免极点附近的问题
            target_lon = own_lon + distance_degrees * math.sin(bearing_rad) / math.cos(math.radians(own_lat))
        else:
            target_lon = own_lon
        
        # 确保目标船位置在有效范围内
        assume(-90.0 <= target_lat <= 90.0)
        assume(-180.0 <= target_lon <= 180.0)
        
        # 计算目标船航向（使其与本船形成交叉态势）
        # 目标船大致朝向本船方向或交叉方向
        target_heading = (bearing_to_target + 180.0 + target_heading_offset) % 360.0
        
        # 创建目标船状态
        target_ship = ShipState(
            mmsi=987654321,
            latitude=target_lat,
            longitude=target_lon,
            heading=target_heading,
            sog=target_speed
        )
        
        # 判定相遇类型
        encounter_type = determine_encounter_type(own_ship, target_ship)
        
        # 只测试交叉相遇场景
        assume(encounter_type == EncounterType.CROSSING)
        
        # 确定船舶角色
        own_role, target_role = determine_vessel_roles(encounter_type, own_ship, target_ship)
        
        # 只测试本船为让路船的情况
        assume(own_role == VesselRole.GIVE_WAY)
        
        # 应用COLREGS规则
        action = apply_colregs_rule(encounter_type, own_ship, target_ship)
        
        # 验证属性：让路船的航向改变应大于等于15度
        assert action.action_type == ActionType.COURSE_CHANGE, \
            f"让路船应该返回航向改变动作，但得到: {action.action_type}"
        
        assert action.turn_angle is not None, \
            "让路船的转向角度不应为None"
        
        assert action.turn_angle >= 15.0, \
            f"让路船的航向改变应大于等于15度（明显避让），但得到: {action.turn_angle}度"
        
        assert action.turn_direction == TurnDirection.STARBOARD, \
            f"让路船应该向右转向，但得到: {action.turn_direction}"
        
        assert "Rule 15" in action.reason or "让路船" in action.reason, \
            f"动作理由应该包含'Rule 15'或'让路船'，但得到: {action.reason}"


class TestPropertyStandOnMaintainCourse:
    """
    Property 9: 直航船的航向保持
    
    Feature: maritime-collision-avoidance, Property 9: 直航船的航向保持
    Validates: Requirements 3.3
    
    对于任何交叉相遇场景中被判定为直航船的情况，在让路船未采取行动前，
    避碰决策模块应输出保持航向的指令
    """
    
    @given(
        own_lat=st.floats(min_value=-80.0, max_value=80.0, allow_nan=False, allow_infinity=False),
        own_lon=st.floats(min_value=-170.0, max_value=170.0, allow_nan=False, allow_infinity=False),
        own_heading=st.floats(min_value=0.0, max_value=359.999, allow_nan=False, allow_infinity=False),
        own_speed=st.floats(min_value=5.0, max_value=25.0, allow_nan=False, allow_infinity=False),
        distance=st.floats(min_value=0.5, max_value=10.0, allow_nan=False, allow_infinity=False),
        target_speed=st.floats(min_value=5.0, max_value=25.0, allow_nan=False, allow_infinity=False),
        relative_bearing=st.floats(min_value=-112.5, max_value=-5.0, allow_nan=False, allow_infinity=False),
        target_heading_offset=st.floats(min_value=-90.0, max_value=90.0, allow_nan=False, allow_infinity=False)
    )
    def test_property_stand_on_maintain_course(
        self,
        own_lat,
        own_lon,
        own_heading,
        own_speed,
        distance,
        target_speed,
        relative_bearing,
        target_heading_offset
    ):
        """
        属性测试：对于任何交叉相遇场景中的直航船，应输出保持航向的指令
        
        测试策略：
        1. 生成随机的本船状态（位置、航向、速度）
        2. 根据交叉相遇条件生成目标船状态：
           - 目标船在左舷（相对方位-112.5到-5度）
           - 这样本船就是直航船
        3. 验证apply_colregs_rule返回的动作为保持航向
        
        Requirements: 3.3
        """
        # 创建本船状态
        own_ship = ShipState(
            mmsi=123456789,
            latitude=own_lat,
            longitude=own_lon,
            heading=own_heading,
            sog=own_speed
        )
        
        # 计算目标船位置（在本船左舷，相对方位在-112.5到-5度之间）
        bearing_to_target = own_heading + relative_bearing
        
        # 将距离从海里转换为度（1度约60海里）
        distance_degrees = distance / 60.0
        
        # 计算目标船的纬度和经度
        bearing_rad = math.radians(bearing_to_target)
        target_lat = own_lat + distance_degrees * math.cos(bearing_rad)
        
        # 考虑纬度修正
        if abs(own_lat) < 89.0:  # 避免极点附近的问题
            target_lon = own_lon + distance_degrees * math.sin(bearing_rad) / math.cos(math.radians(own_lat))
        else:
            target_lon = own_lon
        
        # 确保目标船位置在有效范围内
        assume(-90.0 <= target_lat <= 90.0)
        assume(-180.0 <= target_lon <= 180.0)
        
        # 计算目标船航向（使其与本船形成交叉态势）
        # 目标船大致朝向本船方向或交叉方向
        target_heading = (bearing_to_target + 180.0 + target_heading_offset) % 360.0
        
        # 创建目标船状态
        target_ship = ShipState(
            mmsi=987654321,
            latitude=target_lat,
            longitude=target_lon,
            heading=target_heading,
            sog=target_speed
        )
        
        # 判定相遇类型
        encounter_type = determine_encounter_type(own_ship, target_ship)
        
        # 只测试交叉相遇场景
        assume(encounter_type == EncounterType.CROSSING)
        
        # 确定船舶角色
        own_role, target_role = determine_vessel_roles(encounter_type, own_ship, target_ship)
        
        # 只测试本船为直航船的情况
        assume(own_role == VesselRole.STAND_ON)
        
        # 应用COLREGS规则
        action = apply_colregs_rule(encounter_type, own_ship, target_ship)
        
        # 验证属性：直航船应保持航向和航速
        assert action.action_type == ActionType.MAINTAIN, \
            f"直航船应该返回保持航向动作，但得到: {action.action_type}"
        
        assert action.maintain_course == True, \
            f"直航船应该设置maintain_course=True，但得到: {action.maintain_course}"
        
        assert action.turn_direction is None or action.turn_direction == TurnDirection.NONE, \
            f"直航船不应该转向，但得到转向方向: {action.turn_direction}"
        
        assert action.turn_angle is None or action.turn_angle == 0, \
            f"直航船不应该有转向角度，但得到: {action.turn_angle}"
        
        assert "Rule 17" in action.reason or "直航船" in action.reason, \
            f"动作理由应该包含'Rule 17'或'直航船'，但得到: {action.reason}"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
