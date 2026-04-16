"""
基于主航线的场景编排器
"""

from dataclasses import dataclass
from typing import List, Tuple
import math

from .models import (
    RouteScenarioConfig,
    ScenarioConfig,
    ScenarioType,
    ShipState,
    ShipRole,
)


@dataclass
class RouteAnchorPoint:
    latitude: float
    longitude: float
    heading: float
    segment_index: int


class RouteScenarioGenerator:
    """根据本船主航线生成多 encounter 综合场景。"""

    METERS_PER_NM = 1852.0

    def generate(self, config: RouteScenarioConfig) -> ScenarioConfig:
        own_ship = ShipState.from_dict(config.own_ship.to_dict())
        own_ship.role = ShipRole.OWN_SHIP
        own_ship.collision_avoidance_enabled = True
        own_ship.waypoints = [list(point) for point in config.own_ship_route]

        ships = [own_ship]
        for encounter in config.encounters:
            anchor = self._resolve_anchor(config.own_ship_route, encounter.anchor)
            if encounter.encounter_type == ScenarioType.OVERTAKING:
                ships.append(self._build_overtaking_target(encounter, anchor, config.own_ship_route))
            elif encounter.encounter_type == ScenarioType.CROSSING:
                ships.append(self._build_crossing_target(encounter, anchor))
            elif encounter.encounter_type == ScenarioType.HEAD_ON:
                ships.append(self._build_head_on_target(encounter, anchor, config.own_ship_route))
            else:
                raise ValueError(f"route-aware 暂不支持的 encounter 类型: {encounter.encounter_type.value}")

        return ScenarioConfig(
            scenario_id=config.scenario_id,
            scenario_type=ScenarioType.ROUTE_AWARE,
            ships=ships,
            environment=config.environment,
            duration=config.duration,
            description=config.description or "基于主航线的综合场景",
            success_criteria={"collision_avoided": True},
        )

    def _resolve_anchor(self, route: List[List[float]], anchor_ref) -> RouteAnchorPoint:
        if anchor_ref.route_segment_index is not None:
            idx = min(anchor_ref.route_segment_index, len(route) - 2)
            start = route[idx]
            end = route[idx + 1]
            heading = self._bearing(start[0], start[1], end[0], end[1])
            return RouteAnchorPoint(start[0], start[1], heading, idx)

        progress = anchor_ref.route_progress or 0.0
        cumulative = [0.0]
        total = 0.0
        for i in range(len(route) - 1):
            seg = self._distance_m(route[i][0], route[i][1], route[i + 1][0], route[i + 1][1])
            total += seg
            cumulative.append(total)
        if total <= 0:
            raise ValueError("主航线长度不能为0")
        target = total * progress
        for i in range(len(route) - 1):
            seg_start = cumulative[i]
            seg_end = cumulative[i + 1]
            if target <= seg_end:
                ratio = 0.0 if seg_end == seg_start else (target - seg_start) / (seg_end - seg_start)
                lat = route[i][0] + (route[i + 1][0] - route[i][0]) * ratio
                lon = route[i][1] + (route[i + 1][1] - route[i][1]) * ratio
                heading = self._bearing(route[i][0], route[i][1], route[i + 1][0], route[i + 1][1])
                return RouteAnchorPoint(lat, lon, heading, i)
        start = route[-2]
        end = route[-1]
        return RouteAnchorPoint(end[0], end[1], self._bearing(start[0], start[1], end[0], end[1]), len(route) - 2)

    def _build_overtaking_target(self, encounter, anchor: RouteAnchorPoint, route: List[List[float]]) -> ShipState:
        lat, lon = self._offset_along_track(anchor.latitude, anchor.longitude, anchor.heading, -encounter.relative_distance_nm)
        future_route = [
            [lat, lon],
            *route[anchor.segment_index + 1:anchor.segment_index + 4],
        ]
        return ShipState(
            mmsi=encounter.target_mmsi,
            latitude=lat,
            longitude=lon,
            heading=anchor.heading,
            sog=encounter.target_speed,
            rot=0.0,
            waypoints=future_route,
            start_time=encounter.start_time,
            end_time=encounter.end_time,
            role=ShipRole.TARGET_SHIP,
            encounter_type=encounter.encounter_type.value,
            collision_avoidance_enabled=False,
            route_id=encounter.route_id,
        )

    def _build_crossing_target(self, encounter, anchor: RouteAnchorPoint) -> ShipState:
        side_sign = 1.0 if encounter.crossing_side == "starboard" else -1.0
        offset_bearing = (anchor.heading + side_sign * 90.0) % 360.0
        start_lat, start_lon = self._offset_along_track(
            anchor.latitude, anchor.longitude, offset_bearing, encounter.relative_distance_nm
        )
        target_heading = (anchor.heading - side_sign * encounter.crossing_angle) % 360.0
        end_lat, end_lon = self._offset_along_track(
            anchor.latitude, anchor.longitude, (offset_bearing + 180.0) % 360.0, encounter.relative_distance_nm
        )
        return ShipState(
            mmsi=encounter.target_mmsi,
            latitude=start_lat,
            longitude=start_lon,
            heading=target_heading,
            sog=encounter.target_speed,
            rot=0.0,
            waypoints=[[anchor.latitude, anchor.longitude], [end_lat, end_lon]],
            start_time=encounter.start_time,
            end_time=encounter.end_time,
            role=ShipRole.TARGET_SHIP,
            encounter_type=encounter.encounter_type.value,
            collision_avoidance_enabled=False,
            route_id=encounter.route_id,
        )

    def _build_head_on_target(self, encounter, anchor: RouteAnchorPoint, route: List[List[float]]) -> ShipState:
        start_lat, start_lon = self._offset_along_track(
            anchor.latitude, anchor.longitude, anchor.heading, encounter.relative_distance_nm
        )
        target_heading = (anchor.heading + 180.0) % 360.0
        reverse_points = list(reversed(route[max(0, anchor.segment_index - 2):anchor.segment_index + 1]))
        waypoints = [[anchor.latitude, anchor.longitude], *reverse_points]
        return ShipState(
            mmsi=encounter.target_mmsi,
            latitude=start_lat,
            longitude=start_lon,
            heading=target_heading,
            sog=encounter.target_speed,
            rot=0.0,
            waypoints=waypoints,
            start_time=encounter.start_time,
            end_time=encounter.end_time,
            role=ShipRole.TARGET_SHIP,
            encounter_type=encounter.encounter_type.value,
            collision_avoidance_enabled=False,
            route_id=encounter.route_id,
        )

    def _offset_along_track(self, lat: float, lon: float, bearing_deg: float, distance_nm: float) -> Tuple[float, float]:
        distance_m = distance_nm * self.METERS_PER_NM
        radius = 6371000.0
        brng = math.radians(bearing_deg)
        lat1 = math.radians(lat)
        lon1 = math.radians(lon)
        ang_dist = distance_m / radius
        lat2 = math.asin(math.sin(lat1) * math.cos(ang_dist) + math.cos(lat1) * math.sin(ang_dist) * math.cos(brng))
        lon2 = lon1 + math.atan2(
            math.sin(brng) * math.sin(ang_dist) * math.cos(lat1),
            math.cos(ang_dist) - math.sin(lat1) * math.sin(lat2),
        )
        return math.degrees(lat2), math.degrees(lon2)

    def _bearing(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        diff_lon = math.radians(lon2 - lon1)
        x = math.sin(diff_lon) * math.cos(lat2_rad)
        y = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(diff_lon)
        return (math.degrees(math.atan2(x, y)) + 360.0) % 360.0

    def _distance_m(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        r = 6371000.0
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)
        a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
        return 2 * r * math.atan2(math.sqrt(a), math.sqrt(1 - a))
