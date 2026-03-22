#!/usr/bin/env python3
"""
海事碰撞避免系统 — 可视化演示

独立运行（无需 ROS2），通过浏览器实时展示：
  - 场景构建（对遇 / 交叉 / 追越 / 包围 / 受限水域 / 恶劣天气 / 能见度不良）
  - 船舶实时运动仿真
  - 碰撞风险评估（DCPA / TCPA / CRI）
  - COLREGS 规则判定
  - 自动避碰决策与路径执行

运行方式：
  python3 visual_demo.py
  浏览器打开 http://localhost:5000
"""

import sys
import json
import math
import time
import threading
import copy
from pathlib import Path
from typing import List, Dict, Optional

# --------------------------------------------------------------------------
# 路径注入（无需 colcon build）
# --------------------------------------------------------------------------
WS = Path(__file__).parent
for pkg in ('scenario_generator', 'collision_avoidance', 'data_logger'):
    sys.path.insert(0, str(WS / 'src' / pkg))

from scenario_generator.generator import (
    ScenarioGenerator,
    HeadOnParams, SurroundingParams, RoughWeatherParams,
    PoorVisibilityParams, RestrictedWaterParams, OvertakingParams,
)
from scenario_generator.models import ShipState, ScenarioConfig
from collision_avoidance.risk_assessment import assess_collision_risk
from collision_avoidance.rules_engine import (
    determine_encounter_type, apply_colregs_rule,
    EncounterType, ActionType, TurnDirection, AvoidanceAction, is_overtaking_complete,
)

try:
    from flask import Flask, render_template, Response, request, jsonify
except ImportError:
    print("[错误] 缺少 Flask，请执行: pip install flask")
    sys.exit(1)

app = Flask(__name__)

# --------------------------------------------------------------------------
# 全局仿真状态
# --------------------------------------------------------------------------
_sim_lock = threading.Lock()
_sim_state: Dict = {
    'running': False,
    'tick': 0,
    'ships': [],          # List[dict]  — 当前帧船舶状态
    'risks': [],          # List[dict]  — 当前帧风险评估
    'events': [],         # List[str]   — 最近事件日志
    'scenario_name': '',
    'scenario_desc': '',
    'environment': {},
    'map_polygon': None,  # 受限水域等：[[lat,lon], ...] 闭合多边形，供前端绘制
    'dt': 1.0,            # 仿真步长（秒）
    'total_ticks': 0,
}
_sse_clients: List = []
_sim_thread: Optional[threading.Thread] = None

# 演示用避让：多船时若仍按 warning 频繁转向，几何会反复切换「最大 CRI 目标」→ 本船原地打转
AVOID_COOLDOWN_AFTER_TURN = 80          # 仿真步（≈秒），两次转向之间的最小间隔
MULTI_SHIP_MIN_TARGETS = 3            # 他船 ≥3 艘时启用多船严格策略（包围等）
MULTI_WARN_MIN_CRI = 0.72             # 多船时 warning 须达到该 CRI 才允许转向
MULTI_WARN_MAX_TCPA_MIN = 38.0        # 多船时 warning 还须 TCPA 较紧迫（分钟）才转向；危险档不受限
MIN_HEADING_CHANGE_DEG = 8.0          # 建议航向与当前航向差过小则不下发，避免微调循环

# --------------------------------------------------------------------------
# 场景预设
# --------------------------------------------------------------------------
SCENARIOS = {
    'head_on_basic': {
        'label': '基础对遇',
        'desc': '两艘船以10节相向航行，距离3海里，将触发Rule 14右转避让',
        'env': {'天气': '平静', '能见度': '良好'},
    },
    'head_on_high_speed': {
        'label': '高速对遇',
        'desc': '两艘船以15节高速相向，距离5海里，紧迫性更高',
        'env': {'天气': '平静', '能见度': '良好'},
    },
    'overtaking': {
        'label': '追越避让',
        'desc': '本船18节追越6节慢船，初始距离0.8海里，CRI>0.25提前触发Rule 13侧向避让',
        'env': {'天气': '平静', '能见度': '良好'},
    },
    'overtaking': {
        'label': '追越',
        'desc': '本船从后方追越前船，Rule 13，两船同向',
        'env': {'天气': '平静', '能见度': '良好'},
    },
    'surrounding': {
        'label': '多船包围',
        'desc': '本船被4艘船从四周约3.5海里对称包围（正交方位），测试多威胁决策',
        'env': {'天气': '平静', '能见度': '良好'},
    },
    'rough_weather': {
        'label': '恶劣天气对遇',
        'desc': '15m/s大风、1.5m/s海流环境下的对遇场景',
        'env': {'天气': '恶劣', '风速': '15 m/s', '流速': '1.5 m/s'},
    },
    'poor_visibility': {
        'label': '能见度不良',
        'desc': '能见度仅1海里，系统启用保守风险阈值（Rule 19）',
        'env': {'天气': '大雾', '能见度': '1.0 nm'},
    },
    'restricted_water': {
        'label': '受限水域',
        'desc': '弯曲狭水道（不规则多边形边界），对遇场景，适用 Rule 9',
        'env': {'水域': '狭水道', '航道宽': '0.5 nm'},
    },
}

# --------------------------------------------------------------------------
# 场景生成
# --------------------------------------------------------------------------
gen = ScenarioGenerator()

def _build_scenario(name: str) -> ScenarioConfig:
    if name == 'head_on_basic':
        return gen.generate_head_on_scenario(
            HeadOnParams(distance=3.0, speed1=10.0, speed2=10.0))
    if name == 'head_on_high_speed':
        return gen.generate_head_on_scenario(
            HeadOnParams(distance=5.0, speed1=15.0, speed2=15.0))
    if name == 'overtaking':
        # 本船=快船（追越船），目标=慢船（被追越），东西方向追越更直观
        # 两船均向东（heading=90°），本船在西侧(后方)追目标在东侧(前方)
        scenario = gen.generate_overtaking_scenario(
            OvertakingParams(distance=0.8, speed1=18.0, speed2=6.0,
                             base_latitude=30.0, base_longitude=120.0))
        # generator 生成：ship[0]=追越船(快) 在南，ship[1]=被追越(慢) 在北
        # 改为东西方向：手动调整坐标，让本船在西、目标在东，均朝东航行
        s0, s1 = scenario.ships[0], scenario.ships[1]
        import dataclasses
        dist_deg = 0.8 / 60.0
        scenario.ships[0] = dataclasses.replace(s0,
            latitude=30.0, longitude=120.0 - dist_deg, heading=90.0)
        scenario.ships[1] = dataclasses.replace(s1,
            latitude=30.0, longitude=120.0, heading=90.0)
        return scenario
    if name == 'surrounding':
        # 略大包围半径 + 固定方位角，避免四船过近叠在一起；仍朝本船航行
        return gen.generate_surrounding_scenario(
            SurroundingParams(
                num_surrounding=4,
                surrounding_distance=3.5,
                own_speed=10.0,
                random_offset=False,
                seed=42,
            ))
    if name == 'rough_weather':
        return gen.generate_rough_weather_scenario(
            RoughWeatherParams(wind_speed_ms=15.0, wind_direction=45.0,
                               current_speed_ms=1.5, current_direction=90.0,
                               visibility_nm=3.0))
    if name == 'poor_visibility':
        return gen.generate_poor_visibility_scenario(
            PoorVisibilityParams(visibility_nm=1.0, distance_nm=4.0,
                                 risk_threshold_warning=0.3,
                                 risk_threshold_danger=0.5))
    if name == 'restricted_water':
        return gen.generate_restricted_water_scenario(
            RestrictedWaterParams(channel_width_nm=0.5, channel_length_nm=5.0,
                                  channel_heading=90.0,
                                  own_speed=8.0, target_speed=8.0))
    # 默认
    return gen.generate_head_on_scenario(HeadOnParams())


# --------------------------------------------------------------------------
# 物理仿真辅助
# --------------------------------------------------------------------------
NM_PER_DEG_LAT = 60.0   # 1纬度 ≈ 60海里


def _shortest_heading_diff_deg(from_h: float, to_h: float) -> float:
    """from_h → to_h 的最小有向角差（度），范围 (-180, 180]。"""
    d = (to_h - from_h) % 360.0
    if d > 180.0:
        d -= 360.0
    return d


def _point_in_polygon(lat: float, lon: float, poly: List) -> bool:
    """射线法判断点是否在多边形内；poly 为 [[lat, lon], ...]。"""
    if not poly or len(poly) < 3:
        return True
    n = len(poly)
    inside = False
    j = n - 1
    for i in range(n):
        yi, xi = float(poly[i][0]), float(poly[i][1])
        yj, xj = float(poly[j][0]), float(poly[j][1])
        denom = (yj - yi) or 1e-30
        if ((yi > lat) != (yj > lat)) and (
            lon < (xj - xi) * (lat - yi) / denom + xi
        ):
            inside = not inside
        j = i
    return inside


def _closest_point_on_segment_latlon(
    lat: float, lon: float,
    alat: float, alon: float, blat: float, blon: float,
) -> tuple:
    """线段上距离 (lat,lon) 最近的点，返回 (lat, lon)。"""
    px, py = lon, lat
    ax, ay = alon, alat
    bx, by = blon, blat
    abx, aby = bx - ax, by - ay
    apx, apy = px - ax, py - ay
    ab2 = abx * abx + aby * aby
    if ab2 < 1e-20:
        return alat, alon
    t = max(0.0, min(1.0, (apx * abx + apy * aby) / ab2))
    cx = ax + t * abx
    cy = ay + t * aby
    return cy, cx


def _move_nm_from(lat: float, lon: float, bearing_deg: float, dist_nm: float) -> tuple:
    """沿航海向 bearing_deg（北=0°顺时针）平移 dist_nm 海里。"""
    lat_corr = max(math.cos(math.radians(lat)), 1e-6)
    br = math.radians(bearing_deg)
    dlat = dist_nm * math.cos(br) / NM_PER_DEG_LAT
    dlon = dist_nm * math.sin(br) / (NM_PER_DEG_LAT * lat_corr)
    return lat + dlat, lon + dlon


def _clamp_ship_to_channel(ship: dict, poly: List) -> dict:
    """将船位约束在航道多边形内：最近边界点 + 向内法向推进（质心法在狭弯航道常失效）。"""
    lat, lon = float(ship['latitude']), float(ship['longitude'])
    if _point_in_polygon(lat, lon, poly):
        return ship
    s = copy.deepcopy(ship)
    lat_corr = max(math.cos(math.radians(lat)), 1e-6)
    n = len(poly)

    best_lat, best_lon = lat, lon
    best_d2 = float("inf")
    best_edge = 0
    for i in range(n):
        a = poly[i]
        b = poly[(i + 1) % n]
        alat, alon = float(a[0]), float(a[1])
        blat, blon = float(b[0]), float(b[1])
        clat, clon = _closest_point_on_segment_latlon(lat, lon, alat, alon, blat, blon)
        dlat = (lat - clat) * 60.0
        dlon = (lon - clon) * lat_corr * 60.0
        d2 = dlat * dlat + dlon * dlon
        if d2 < best_d2:
            best_d2 = d2
            best_lat, best_lon = clat, clon
            best_edge = i

    a = poly[best_edge]
    b = poly[(best_edge + 1) % n]
    alat, alon = float(a[0]), float(a[1])
    blat, blon = float(b[0]), float(b[1])
    vx = blon - alon
    vy = blat - alat
    nxlon = -vy
    nlatc = vx
    norm = math.hypot(nxlon, nlatc) or 1e-12
    nxlon /= norm
    nlatc /= norm
    mlat = (alat + blat) / 2.0
    mlon = (alon + blon) / 2.0
    clats = sum(float(p[0]) for p in poly) / n
    clons = sum(float(p[1]) for p in poly) / n
    if nxlon * (clons - mlon) + nlatc * (clats - mlat) < 0:
        nxlon, nlatc = -nxlon, -nlatc

    n_east = nxlon * lat_corr
    n_north = nlatc
    brg = math.degrees(math.atan2(n_east, n_north)) % 360.0

    lat_s, lon_s = best_lat, best_lon
    for _ in range(120):
        if _point_in_polygon(lat_s, lon_s, poly):
            s["latitude"], s["longitude"] = lat_s, lon_s
            return s
        lat_s, lon_s = _move_nm_from(lat_s, lon_s, brg, 0.04)

    lat_s, lon_s = lat, lon
    for _ in range(200):
        if _point_in_polygon(lat_s, lon_s, poly):
            s["latitude"], s["longitude"] = lat_s, lon_s
            return s
        lat_s, lon_s = _move_nm_from(lat_s, lon_s, brg, 0.05)

    s["latitude"], s["longitude"] = best_lat, best_lon
    return s


def _warrants_course_change(ra, num_targets: int, enc_type=None) -> bool:
    """
    是否应把该目标纳入「可转向避让」候选。
    多船时：仅对 danger 或高 CRI+较紧迫 TCPA 的 warning 做转向，避免长期 warning 抖动绕圈。
    追越场景：DCPA 接近 0 但 TCPA 可能较大，提前到 CRI>0.35 就触发。
    """
    # 追越场景优先判断：CRI > 0.25 且 TCPA > 0 即触发，不受 warning 阈值限制
    # 若 DCPA 已经 > 0.3nm，说明已经绕开，不再重复触发
    if enc_type is not None and enc_type.value == 'overtaking':
        if ra.dcpa > 0.3:
            return False
        return ra.cri > 0.25 and ra.tcpa > 0
    if ra.risk_level.value == 'danger':
        return True
    if ra.risk_level.value != 'warning':
        return False
    if num_targets < MULTI_SHIP_MIN_TARGETS:
        return True
    if ra.cri < MULTI_WARN_MIN_CRI:
        return False
    t = ra.tcpa
    if t <= 0 or t > MULTI_WARN_MAX_TCPA_MIN:
        return False
    return True


def _move_ship(ship: dict, dt: float) -> dict:
    """按当前航向/航速推进一步（dt 秒）。"""
    s = copy.deepcopy(ship)
    dist_nm = s['sog'] * (dt / 3600.0)   # 节 → 海里/秒
    heading_rad = math.radians(s['heading'])
    lat_nm_per_deg = NM_PER_DEG_LAT
    cos_lat = math.cos(math.radians(s['latitude']))
    lon_nm_per_deg = NM_PER_DEG_LAT * cos_lat if cos_lat > 1e-6 else NM_PER_DEG_LAT

    dlat = dist_nm * math.cos(heading_rad) / lat_nm_per_deg
    dlon = dist_nm * math.sin(heading_rad) / lon_nm_per_deg
    s['latitude'] += dlat
    s['longitude'] += dlon
    # 平滑转向：逐步逼近目标航向
    if 'target_heading' in s and s['target_heading'] is not None:
        diff = s['target_heading'] - s['heading']
        # 归一化
        while diff > 180: diff -= 360
        while diff < -180: diff += 360
        step = min(3.0, abs(diff)) * (1 if diff > 0 else -1)
        s['heading'] = (s['heading'] + step) % 360
        if abs(diff) < 1.0:
            s['target_heading'] = None
            # 完成一次转向后短暂冷却，避免多目标/几何抖动导致每帧改目标航向 → 原地打转
            if s.get('is_own'):
                s['avoidance_cooldown'] = max(
                    s.get('avoidance_cooldown', 0), AVOID_COOLDOWN_AFTER_TURN
                )
    return s


def _ship_to_state(d: dict) -> ShipState:
    return ShipState(
        mmsi=int(d['mmsi']),
        latitude=float(d['latitude']),
        longitude=float(d['longitude']),
        heading=float(d['heading']) % 360,
        sog=float(d['sog']),
        rot=float(d.get('rot', 0.0)),
    )


def _assess_all(ships: List[dict]):
    """评估所有船对的风险，返回风险列表和事件列表。"""
    risks = []
    events = []
    own = ships[0]  # 第一艘始终是本船
    own_state = _ship_to_state(own)
    num_targets = len(ships) - 1

    # 冷却递减（仿真步内每 tick 一次）
    cd = own.get('avoidance_cooldown', 0)
    if cd > 0:
        own['avoidance_cooldown'] = cd - 1

    avoidance_candidates = []

    for i, tgt in enumerate(ships[1:], 1):
        try:
            tgt_state = _ship_to_state(tgt)
            ra = assess_collision_risk(own_state, tgt_state)
            enc = determine_encounter_type(own_state, tgt_state)

            action = apply_colregs_rule(enc, own_state, tgt_state)

            risk_entry = {
                'target_idx': i,
                'target_mmsi': tgt['mmsi'],
                'dcpa': round(ra.dcpa, 3),
                'tcpa': round(ra.tcpa, 2),
                'cri': round(ra.cri, 3),
                'risk_level': ra.risk_level.value,
                'distance': round(ra.distance, 3),
                'encounter_type': enc.value,
                'action_type': action.action_type.value,
                'turn_direction': action.turn_direction.value if action.turn_direction else 'none',
                'turn_angle': action.turn_angle or 0,
                'action_reason': action.reason,
            }
            risks.append(risk_entry)

            if ra.risk_level.value in ('warning', 'danger'):
                if (
                    action.action_type == ActionType.COURSE_CHANGE
                    and own.get('target_heading') is None
                    and _warrants_course_change(ra, num_targets, enc)
                ):
                    avoidance_candidates.append((i, tgt, ra, action))
                elif ra.risk_level.value == 'danger':
                    events.append(
                        f"[危险] MMSI {own['mmsi']} ↔ MMSI {tgt['mmsi']} "
                        f"CRI={ra.cri:.3f} DCPA={ra.dcpa:.2f}nm TCPA={ra.tcpa:.1f}min"
                    )

            # 追越完成：规则引擎返回 RESUME_COURSE，回正到原始航向
            # 注意：回正不受 avoidance_cooldown 限制（这是主动安全恢复动作）
            if (action.action_type == ActionType.RESUME_COURSE
                    and own.get('target_heading') is None):
                orig = own.get('original_heading')
                if orig is not None:
                    diff = abs(_shortest_heading_diff_deg(own['heading'], orig))
                    if diff > 2.0:  # 当前航向已偏离原始航向才回正
                        own['target_heading'] = orig
                        own['avoidance_cooldown'] = 0  # 清除冷却，允许立即回正
                        events.append(
                            f"[回正] MMSI {own['mmsi']} 追越完成，回正航向 {orig:.0f}°"
                        )
        except Exception:
            pass

    # 仅对 CRI 最高者下发一次转向，避免「列表第一个目标」与多目标指令来回打架导致绕圈
    if (
        avoidance_candidates
        and own.get('target_heading') is None
        and own.get('avoidance_cooldown', 0) == 0
    ):
        _i, tgt, ra, action = max(avoidance_candidates, key=lambda x: x[2].cri)
        delta = action.turn_angle or 15.0
        if action.turn_direction == TurnDirection.PORT:
            delta = -delta
        new_hdg = (own['heading'] + delta) % 360
        small_turn = (
            abs(_shortest_heading_diff_deg(own['heading'], new_hdg)) < MIN_HEADING_CHANGE_DEG
        )
        if small_turn and ra.risk_level.value != 'danger':
            pass
        else:
            own['target_heading'] = new_hdg
            own['original_heading'] = own.get('original_heading', own['heading'])
            events.append(
                f"[避碰] MMSI {own['mmsi']} → 目标航向 {new_hdg:.0f}° "
                f"({action.reason.split('：')[-1] if '：' in action.reason else action.reason})"
            )

    return risks, events


# --------------------------------------------------------------------------
# 仿真主循环
# --------------------------------------------------------------------------
def _sim_loop():
    global _sim_state
    with _sim_lock:
        ships = _sim_state['ships']
        dt = _sim_state['dt']

    tick = 0
    max_ticks = _sim_state['total_ticks']
    event_log = []

    while True:
        with _sim_lock:
            if not _sim_state['running']:
                break

        # 推进每艘船
        ships = [_move_ship(s, dt) for s in ships]

        with _sim_lock:
            poly = _sim_state.get('map_polygon')
        if poly and len(poly) >= 3:
            ships = [_clamp_ship_to_channel(s, poly) for s in ships]

        # 风险评估 + 避碰决策
        risks, new_events = _assess_all(ships)
        event_log = (new_events + event_log)[:30]  # 最多30条

        tick += 1
        elapsed = tick * dt  # 已仿真秒数

        # 更新全局状态
        with _sim_lock:
            _sim_state['ships'] = ships
            _sim_state['risks'] = risks
            _sim_state['events'] = event_log
            _sim_state['tick'] = tick

        # 推送 SSE
        _broadcast()

        # 到达最大时长则停止
        if max_ticks > 0 and tick >= max_ticks:
            with _sim_lock:
                _sim_state['running'] = False
                _sim_state['events'] = ['[完成] 场景仿真结束'] + event_log
            _broadcast()
            break

        time.sleep(0.25)  # 4x 加速（每0.25s推进1s仿真时间）


# --------------------------------------------------------------------------
# SSE 广播
# --------------------------------------------------------------------------
def _broadcast():
    with _sim_lock:
        payload = {
            'tick': _sim_state['tick'],
            'running': _sim_state['running'],
            'ships': _sim_state['ships'],
            'risks': _sim_state['risks'],
            'events': _sim_state['events'][:10],
            'scenario_name': _sim_state['scenario_name'],
            'map_polygon': _sim_state.get('map_polygon'),
        }
    data = f"data: {json.dumps(payload, ensure_ascii=False)}\n\n"
    dead = []
    for q in _sse_clients:
        try:
            q.put_nowait(data)
        except Exception:
            dead.append(q)
    for q in dead:
        try:
            _sse_clients.remove(q)
        except ValueError:
            pass


# --------------------------------------------------------------------------
# Flask 路由
# --------------------------------------------------------------------------
@app.route('/')
def index():
    return render_template('index.html', scenarios=SCENARIOS)


@app.route('/api/scenarios')
def api_scenarios():
    return jsonify(SCENARIOS)


@app.route('/api/start', methods=['POST'])
def api_start():
    global _sim_thread
    data = request.get_json(force=True)
    name = data.get('scenario', 'head_on_basic')
    speed = float(data.get('speed', 4))  # 仿真加速倍数

    # 停止旧仿真
    with _sim_lock:
        _sim_state['running'] = False
    if _sim_thread and _sim_thread.is_alive():
        _sim_thread.join(timeout=2)

    # 构建场景
    try:
        scenario = _build_scenario(name)
    except Exception as e:
        return jsonify({'error': str(e)}), 400

    # 初始化仿真状态
    ship_labels = None
    if name == 'overtaking':
        ship_labels = ['本船(追越)', '被追越']
    elif name == 'restricted_water':
        ship_labels = ['本船', '目标(对遇)']

    ships_data = []
    for i, ship in enumerate(scenario.ships):
        if ship_labels and i < len(ship_labels):
            lbl = ship_labels[i]
        else:
            lbl = '本船' if i == 0 else f'目标{i}'
        ships_data.append({
            'mmsi': ship.mmsi,
            'latitude': ship.latitude,
            'longitude': ship.longitude,
            'heading': ship.heading,
            'sog': ship.sog,
            'rot': ship.rot,
            'target_heading': None,
            'avoidance_cooldown': 0,
            'original_heading': ship.heading,  # 记录初始航向，追越完成后回正用
            'is_own': i == 0,
            'label': lbl,
        })

    env = scenario.environment
    env_info = {
        '天气': env.weather_condition.value,
        '能见度': env.visibility.value,
        '风速': f'{env.wind_speed:.1f} m/s',
        '流速': f'{env.current_speed:.1f} m/s',
        '水域': env.water_area_type.value,
    }

    dt = 1.0  # 仿真步长（秒）
    # 按场景类型设置合理仿真时长，避免演示时间过长
    SCENE_DURATION = {
        'head_on_basic':    180,
        'head_on_high_speed': 150,
        'overtaking':       500,
        'surrounding':      200,
        'rough_weather':    180,
        'poor_visibility':  180,
        'restricted_water': 200,
    }
    total_ticks = SCENE_DURATION.get(name, min(int(scenario.duration / dt), 240))

    map_polygon = None
    mb = scenario.environment.map_boundaries if scenario.environment else None
    if mb and len(mb) >= 3:
        map_polygon = [[float(lat), float(lon)] for lat, lon in mb]

    with _sim_lock:
        _sim_state.update({
            'running': True,
            'tick': 0,
            'ships': ships_data,
            'risks': [],
            'events': [f'[开始] 场景: {SCENARIOS.get(name, {}).get("label", name)}'],
            'scenario_name': name,
            'scenario_desc': scenario.description,
            'environment': env_info,
            'map_polygon': map_polygon,
            'dt': dt,
            'total_ticks': total_ticks,
        })

    _sim_thread = threading.Thread(target=_sim_loop, daemon=True)
    _sim_thread.start()

    return jsonify({'status': 'started', 'scenario': name,
                    'ships': len(ships_data), 'duration': scenario.duration})


@app.route('/api/stop', methods=['POST'])
def api_stop():
    with _sim_lock:
        _sim_state['running'] = False
        _sim_state['events'] = ['[停止] 仿真已手动停止'] + _sim_state['events']
    _broadcast()
    return jsonify({'status': 'stopped'})


@app.route('/api/state')
def api_state():
    with _sim_lock:
        return jsonify(_sim_state)


@app.route('/stream')
def stream():
    import queue
    q = queue.Queue(maxsize=50)
    _sse_clients.append(q)

    def generate():
        # 立即发送当前状态
        with _sim_lock:
            payload = {
                'tick': _sim_state['tick'],
                'running': _sim_state['running'],
                'ships': _sim_state['ships'],
                'risks': _sim_state['risks'],
                'events': _sim_state['events'][:10],
                'scenario_name': _sim_state['scenario_name'],
                'map_polygon': _sim_state.get('map_polygon'),
            }
        yield f"data: {json.dumps(payload, ensure_ascii=False)}\n\n"

        while True:
            try:
                msg = q.get(timeout=30)
                yield msg
            except Exception:
                yield 'data: {"ping":1}\n\n'

    resp = Response(generate(), mimetype='text/event-stream')
    resp.headers['Cache-Control'] = 'no-cache'
    resp.headers['X-Accel-Buffering'] = 'no'
    return resp


# --------------------------------------------------------------------------
# 入口
# --------------------------------------------------------------------------
if __name__ == '__main__':
    print()
    print('=' * 56)
    print('  海事碰撞避免系统 — 可视化演示')
    print('  浏览器打开: http://localhost:5000')
    print('=' * 56)
    print()
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
