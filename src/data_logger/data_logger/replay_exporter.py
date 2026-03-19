"""
回放数据准备模块

将 DataStore 中记录的数据转换为标准回放格式，
支持按时间轴重现船舶运动和避碰决策过程。
Requirements: 7.5
"""

import json
import os
import csv
from datetime import datetime
from typing import List, Optional, Dict, Any

from data_logger.data_models import (
    DataStore, AisRecord, DecisionRecord, RiskRecord
)


class ReplayFrame:
    """
    单帧回放数据，包含某一时刻的所有船舶状态和风险信息
    Requirements: 7.5
    """
    __slots__ = ['timestamp', 'ships', 'risks', 'decisions']

    def __init__(self, timestamp: float):
        self.timestamp: float = timestamp
        self.ships: List[Dict[str, Any]] = []      # 该帧所有船舶状态
        self.risks: List[Dict[str, Any]] = []      # 该帧所有风险记录
        self.decisions: List[Dict[str, Any]] = []  # 该帧所有决策

    def to_dict(self) -> Dict[str, Any]:
        return {
            'timestamp': self.timestamp,
            'ships': self.ships,
            'risks': self.risks,
            'decisions': self.decisions,
        }


class ReplayData:
    """
    完整回放数据集
    包含元数据和按时间排序的帧序列
    Requirements: 7.5
    """

    def __init__(self, scenario_id: str = '', description: str = ''):
        self.scenario_id = scenario_id
        self.description = description
        self.created_at = datetime.now().isoformat()
        self.frames: List[ReplayFrame] = []
        self.metadata: Dict[str, Any] = {}

    @property
    def duration(self) -> float:
        """回放总时长（秒）"""
        if len(self.frames) < 2:
            return 0.0
        return self.frames[-1].timestamp - self.frames[0].timestamp

    @property
    def frame_count(self) -> int:
        return len(self.frames)

    @property
    def start_time(self) -> Optional[float]:
        return self.frames[0].timestamp if self.frames else None

    @property
    def end_time(self) -> Optional[float]:
        return self.frames[-1].timestamp if self.frames else None

    def to_dict(self) -> Dict[str, Any]:
        return {
            'scenario_id': self.scenario_id,
            'description': self.description,
            'created_at': self.created_at,
            'duration_sec': round(self.duration, 3),
            'frame_count': self.frame_count,
            'start_time': self.start_time,
            'end_time': self.end_time,
            'metadata': self.metadata,
            'frames': [f.to_dict() for f in self.frames],
        }


class ReplayExporter:
    """
    回放数据导出器

    将 DataStore 中的原始记录重采样为等间隔帧序列，
    并导出为 JSON 或 CSV 格式的回放文件。
    Requirements: 7.5
    """

    def __init__(self, output_dir: str = '/tmp/replay_data'):
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)

    def build_replay(
            self,
            store: DataStore,
            frame_interval: float = 1.0,
            scenario_id: str = '',
            description: str = '',
    ) -> ReplayData:
        """
        从 DataStore 构建回放数据集

        Args:
            store: 数据存储
            frame_interval: 帧间隔（秒），默认1秒
            scenario_id: 场景ID
            description: 场景描述
        Returns:
            ReplayData
        """
        if frame_interval <= 0:
            raise ValueError(f'帧间隔必须大于0: {frame_interval}')

        replay = ReplayData(scenario_id=scenario_id, description=description)

        ais_records = store.get_ais_records()
        risk_records = store.get_risk_records()
        decision_records = store.get_decision_records()

        if not ais_records:
            return replay

        # 确定时间范围
        all_ts = [r.timestamp for r in ais_records]
        t_start = min(all_ts)
        t_end = max(all_ts)

        replay.metadata = {
            'frame_interval_sec': frame_interval,
            'ais_record_count': len(ais_records),
            'risk_record_count': len(risk_records),
            'decision_record_count': len(decision_records),
            't_start': t_start,
            't_end': t_end,
        }

        # 按 MMSI 分组 AIS 记录
        ais_by_mmsi: Dict[int, List[AisRecord]] = {}
        for r in sorted(ais_records, key=lambda x: x.timestamp):
            ais_by_mmsi.setdefault(r.mmsi, []).append(r)

        # 构建帧序列
        t = t_start
        while t <= t_end + frame_interval * 0.5:
            frame = ReplayFrame(timestamp=round(t, 3))

            # 每艘船：取最近的历史记录
            for mmsi, records in ais_by_mmsi.items():
                ship_state = self._nearest_record(records, t)
                if ship_state is not None:
                    frame.ships.append(ship_state.to_dict())

            # 该帧时间窗口内的风险记录
            for r in risk_records:
                if abs(r.timestamp - t) <= frame_interval / 2:
                    frame.risks.append(r.to_dict())

            # 该帧时间窗口内的决策记录
            for d in decision_records:
                if abs(d.timestamp - t) <= frame_interval / 2:
                    frame.decisions.append(d.to_dict())

            replay.frames.append(frame)
            t += frame_interval

        return replay

    def export_json(
            self,
            replay: ReplayData,
            filename: Optional[str] = None,
    ) -> str:
        """
        导出为 JSON 回放文件
        Requirements: 7.5
        """
        filename = filename or self._ts_name('replay', 'json')
        path = os.path.join(self.output_dir, filename)
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(replay.to_dict(), f, ensure_ascii=False, indent=2)
        return path

    def export_csv_frames(
            self,
            replay: ReplayData,
            filename: Optional[str] = None,
    ) -> str:
        """
        导出船舶轨迹为 CSV（每行一个船舶时刻点）
        Requirements: 7.5
        """
        filename = filename or self._ts_name('replay_tracks', 'csv')
        path = os.path.join(self.output_dir, filename)
        rows = []
        for frame in replay.frames:
            for ship in frame.ships:
                rows.append({
                    'frame_timestamp': frame.timestamp,
                    'mmsi': ship.get('mmsi', ''),
                    'latitude': ship.get('latitude', ''),
                    'longitude': ship.get('longitude', ''),
                    'heading': ship.get('heading', ''),
                    'sog': ship.get('sog', ''),
                    'rot': ship.get('rot', ''),
                })
        if not rows:
            open(path, 'w').close()
            return path
        with open(path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=rows[0].keys())
            writer.writeheader()
            writer.writerows(rows)
        return path

    @staticmethod
    def _nearest_record(
            records: List[AisRecord], t: float
    ) -> Optional[AisRecord]:
        """
        从已排序记录中找到时间上最接近 t 且不超过 t 的记录
        """
        result = None
        for r in records:
            if r.timestamp <= t:
                result = r
            else:
                break
        return result

    def _ts_name(self, prefix: str, ext: str) -> str:
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        return f'{prefix}_{ts}.{ext}'
