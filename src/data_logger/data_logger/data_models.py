"""
数据记录核心模型：纯Python，不依赖ROS2
Requirements: 9.1-9.4
"""

import os
import json
import csv
import time
from datetime import datetime
from typing import List, Optional


class AisRecord:
    """AIS 数据记录点 Requirements: 9.1"""
    __slots__ = ['timestamp', 'mmsi', 'latitude', 'longitude',
                 'heading', 'sog', 'rot']

    def __init__(self, timestamp: float, mmsi: int, latitude: float,
                 longitude: float, heading: float, sog: float, rot: float):
        self.timestamp = timestamp
        self.mmsi = mmsi
        self.latitude = latitude
        self.longitude = longitude
        self.heading = heading
        self.sog = sog
        self.rot = rot

    def to_dict(self) -> dict:
        return {
            'timestamp': self.timestamp,
            'mmsi': self.mmsi,
            'latitude': self.latitude,
            'longitude': self.longitude,
            'heading': self.heading,
            'sog': self.sog,
            'rot': self.rot,
        }


class DecisionRecord:
    """避让决策记录点 Requirements: 9.2"""
    __slots__ = ['timestamp', 'own_mmsi', 'decision_type',
                 'target_heading', 'target_speed', 'reason', 'raw']

    def __init__(self, timestamp: float, own_mmsi: int, decision_type: str,
                 target_heading: Optional[float], target_speed: Optional[float],
                 reason: str, raw: str = ''):
        self.timestamp = timestamp
        self.own_mmsi = own_mmsi
        self.decision_type = decision_type
        self.target_heading = target_heading
        self.target_speed = target_speed
        self.reason = reason
        self.raw = raw

    def to_dict(self) -> dict:
        return {
            'timestamp': self.timestamp,
            'own_mmsi': self.own_mmsi,
            'decision_type': self.decision_type,
            'target_heading': self.target_heading,
            'target_speed': self.target_speed,
            'reason': self.reason,
        }


class RiskRecord:
    """中间计算结果记录点 Requirements: 9.3"""
    __slots__ = ['timestamp', 'own_mmsi', 'target_mmsi',
                 'dcpa', 'tcpa', 'cri', 'risk_level']

    def __init__(self, timestamp: float, own_mmsi: int, target_mmsi: int,
                 dcpa: float, tcpa: float, cri: float, risk_level: str):
        self.timestamp = timestamp
        self.own_mmsi = own_mmsi
        self.target_mmsi = target_mmsi
        self.dcpa = dcpa
        self.tcpa = tcpa
        self.cri = cri
        self.risk_level = risk_level

    def to_dict(self) -> dict:
        return {
            'timestamp': self.timestamp,
            'own_mmsi': self.own_mmsi,
            'target_mmsi': self.target_mmsi,
            'dcpa': self.dcpa,
            'tcpa': self.tcpa,
            'cri': self.cri,
            'risk_level': self.risk_level,
        }


class DataStore:
    """
    数据存储 Requirements: 9.1-9.3
    """

    def __init__(self):
        self._ais_records: List[AisRecord] = []
        self._decision_records: List[DecisionRecord] = []
        self._risk_records: List[RiskRecord] = []

    def add_ais(self, record: AisRecord):
        self._ais_records.append(record)

    def get_ais_records(self) -> List[AisRecord]:
        return list(self._ais_records)

    def get_ais_by_mmsi(self, mmsi: int) -> List[AisRecord]:
        return [r for r in self._ais_records if r.mmsi == mmsi]

    def add_decision(self, record: DecisionRecord):
        self._decision_records.append(record)

    def get_decision_records(self) -> List[DecisionRecord]:
        return list(self._decision_records)

    def add_risk(self, record: RiskRecord):
        self._risk_records.append(record)

    def get_risk_records(self) -> List[RiskRecord]:
        return list(self._risk_records)

    @property
    def ais_count(self) -> int:
        return len(self._ais_records)

    @property
    def decision_count(self) -> int:
        return len(self._decision_records)

    @property
    def risk_count(self) -> int:
        return len(self._risk_records)

    def clear(self):
        self._ais_records.clear()
        self._decision_records.clear()
        self._risk_records.clear()


class DataExporter:
    """
    数据导出器：支持 CSV 和 JSON 格式
    Requirements: 9.4
    """

    def __init__(self, output_dir: str):
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)

    def export_ais_csv(self, records: List[AisRecord],
                       filename: Optional[str] = None) -> str:
        filename = filename or self._ts_name('ais', 'csv')
        path = os.path.join(self.output_dir, filename)
        if not records:
            open(path, 'w').close()
            return path
        with open(path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=records[0].to_dict().keys())
            writer.writeheader()
            writer.writerows(r.to_dict() for r in records)
        return path

    def export_decisions_csv(self, records: List[DecisionRecord],
                             filename: Optional[str] = None) -> str:
        filename = filename or self._ts_name('decisions', 'csv')
        path = os.path.join(self.output_dir, filename)
        if not records:
            open(path, 'w').close()
            return path
        with open(path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=records[0].to_dict().keys())
            writer.writeheader()
            writer.writerows(r.to_dict() for r in records)
        return path

    def export_risks_csv(self, records: List[RiskRecord],
                         filename: Optional[str] = None) -> str:
        filename = filename or self._ts_name('risks', 'csv')
        path = os.path.join(self.output_dir, filename)
        if not records:
            open(path, 'w').close()
            return path
        with open(path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=records[0].to_dict().keys())
            writer.writeheader()
            writer.writerows(r.to_dict() for r in records)
        return path

    def export_all_json(self, store: DataStore,
                        filename: Optional[str] = None) -> str:
        filename = filename or self._ts_name('all', 'json')
        path = os.path.join(self.output_dir, filename)
        data = {
            'exported_at': datetime.now().isoformat(),
            'ais_records': [r.to_dict() for r in store.get_ais_records()],
            'decision_records': [r.to_dict() for r in store.get_decision_records()],
            'risk_records': [r.to_dict() for r in store.get_risk_records()],
            'summary': {
                'ais_count': store.ais_count,
                'decision_count': store.decision_count,
                'risk_count': store.risk_count,
            }
        }
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        return path

    def _ts_name(self, prefix: str, ext: str) -> str:
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        return f"{prefix}_{ts}.{ext}"
