"""
回放数据准备模块单元测试
Requirements: 7.5
"""
import os
import json
import csv
import tempfile
import pytest
from hypothesis import given, settings, strategies as st

from data_logger.data_models import (
    DataStore, AisRecord, DecisionRecord, RiskRecord
)
from data_logger.replay_exporter import ReplayExporter, ReplayData, ReplayFrame


# ============================================================================
# 辅助函数
# ============================================================================

def make_store(n_ships=2, n_steps=5, dt=1.0, t0=1000.0) -> DataStore:
    """构建含多船多时刻的 DataStore"""
    store = DataStore()
    for step in range(n_steps):
        t = t0 + step * dt
        for i in range(n_ships):
            store.add_ais(AisRecord(
                timestamp=t,
                mmsi=100000000 + i,
                latitude=30.0 + i * 0.01 + step * 0.001,
                longitude=120.0 + i * 0.01,
                heading=float(90 + i * 10),
                sog=10.0 + i,
                rot=0.0,
            ))
    return store


def make_store_with_all(n_steps=5, dt=1.0, t0=1000.0) -> DataStore:
    store = make_store(2, n_steps, dt, t0)
    for step in range(n_steps):
        t = t0 + step * dt
        store.add_risk(RiskRecord(
            timestamp=t,
            own_mmsi=100000000,
            target_mmsi=100000001,
            dcpa=0.5 - step * 0.05,
            tcpa=10.0 - step,
            cri=0.3 + step * 0.05,
            risk_level='warning',
        ))
        store.add_decision(DecisionRecord(
            timestamp=t,
            own_mmsi=100000000,
            decision_type='course_change',
            target_heading=100.0,
            target_speed=8.0,
            reason='avoid collision',
        ))
    return store


# ============================================================================
# ReplayFrame 测试
# ============================================================================

class TestReplayFrame:
    def test_to_dict_has_required_fields(self):
        frame = ReplayFrame(timestamp=1000.0)
        d = frame.to_dict()
        for k in ['timestamp', 'ships', 'risks', 'decisions']:
            assert k in d

    def test_timestamp_preserved(self):
        frame = ReplayFrame(timestamp=1234.5)
        assert frame.to_dict()['timestamp'] == 1234.5

    def test_empty_lists_by_default(self):
        frame = ReplayFrame(timestamp=0.0)
        assert frame.ships == []
        assert frame.risks == []
        assert frame.decisions == []


# ============================================================================
# ReplayData 测试
# ============================================================================

class TestReplayData:
    def test_duration_empty(self):
        rd = ReplayData()
        assert rd.duration == 0.0

    def test_duration_single_frame(self):
        rd = ReplayData()
        rd.frames.append(ReplayFrame(1000.0))
        assert rd.duration == 0.0

    def test_duration_multiple_frames(self):
        rd = ReplayData()
        rd.frames = [ReplayFrame(1000.0), ReplayFrame(1005.0)]
        assert abs(rd.duration - 5.0) < 1e-9

    def test_frame_count(self):
        rd = ReplayData()
        rd.frames = [ReplayFrame(float(i)) for i in range(10)]
        assert rd.frame_count == 10

    def test_start_end_time(self):
        rd = ReplayData()
        rd.frames = [ReplayFrame(100.0), ReplayFrame(200.0), ReplayFrame(300.0)]
        assert rd.start_time == 100.0
        assert rd.end_time == 300.0

    def test_to_dict_has_required_fields(self):
        rd = ReplayData(scenario_id='test_001', description='test')
        d = rd.to_dict()
        for k in ['scenario_id', 'description', 'created_at',
                  'duration_sec', 'frame_count', 'frames', 'metadata']:
            assert k in d

    def test_scenario_id_preserved(self):
        rd = ReplayData(scenario_id='abc123')
        assert rd.to_dict()['scenario_id'] == 'abc123'


# ============================================================================
# ReplayExporter.build_replay 测试
# ============================================================================

class TestBuildReplay:
    def setup_method(self):
        self.tmp = tempfile.mkdtemp()
        self.exp = ReplayExporter(self.tmp)

    def test_empty_store_returns_empty_replay(self):
        store = DataStore()
        rd = self.exp.build_replay(store)
        assert rd.frame_count == 0

    def test_invalid_interval_raises(self):
        store = make_store()
        with pytest.raises(ValueError):
            self.exp.build_replay(store, frame_interval=0.0)

    def test_frame_count_correct(self):
        store = make_store(2, 5, dt=1.0, t0=1000.0)
        rd = self.exp.build_replay(store, frame_interval=1.0)
        # 5步，时间范围 1000-1004，1秒间隔 -> 5帧
        assert rd.frame_count == 5

    def test_each_frame_has_ships(self):
        store = make_store(2, 5)
        rd = self.exp.build_replay(store, frame_interval=1.0)
        for frame in rd.frames:
            assert len(frame.ships) == 2

    def test_frames_sorted_by_time(self):
        store = make_store(2, 5)
        rd = self.exp.build_replay(store, frame_interval=1.0)
        times = [f.timestamp for f in rd.frames]
        assert times == sorted(times)

    def test_risk_records_in_frames(self):
        store = make_store_with_all(5)
        rd = self.exp.build_replay(store, frame_interval=1.0)
        total_risks = sum(len(f.risks) for f in rd.frames)
        assert total_risks > 0

    def test_decision_records_in_frames(self):
        store = make_store_with_all(5)
        rd = self.exp.build_replay(store, frame_interval=1.0)
        total_decisions = sum(len(f.decisions) for f in rd.frames)
        assert total_decisions > 0

    def test_metadata_set(self):
        store = make_store(2, 5)
        rd = self.exp.build_replay(store, frame_interval=1.0,
                                   scenario_id='s1', description='desc')
        assert rd.scenario_id == 's1'
        assert rd.description == 'desc'
        assert 'frame_interval_sec' in rd.metadata

    def test_scenario_id_in_replay(self):
        store = make_store(2, 5)
        rd = self.exp.build_replay(store, scenario_id='test_scenario')
        assert rd.scenario_id == 'test_scenario'


# ============================================================================
# ReplayExporter.export_json 测试
# ============================================================================

class TestExportJson:
    def setup_method(self):
        self.tmp = tempfile.mkdtemp()
        self.exp = ReplayExporter(self.tmp)

    def test_file_created(self):
        store = make_store(2, 3)
        rd = self.exp.build_replay(store)
        path = self.exp.export_json(rd, 'test_replay.json')
        assert os.path.exists(path)

    def test_json_valid(self):
        store = make_store(2, 3)
        rd = self.exp.build_replay(store)
        path = self.exp.export_json(rd, 'test_replay.json')
        with open(path) as f:
            data = json.load(f)
        assert 'frames' in data
        assert 'frame_count' in data

    def test_json_frame_count_correct(self):
        store = make_store(2, 4, dt=1.0)
        rd = self.exp.build_replay(store, frame_interval=1.0)
        path = self.exp.export_json(rd, 'test_replay.json')
        with open(path) as f:
            data = json.load(f)
        assert data['frame_count'] == rd.frame_count

    def test_custom_filename(self):
        store = make_store()
        rd = self.exp.build_replay(store)
        path = self.exp.export_json(rd, 'custom_name.json')
        assert path.endswith('custom_name.json')

    def test_auto_filename(self):
        store = make_store()
        rd = self.exp.build_replay(store)
        path = self.exp.export_json(rd)
        assert path.endswith('.json')


# ============================================================================
# ReplayExporter.export_csv_frames 测试
# ============================================================================

class TestExportCsv:
    def setup_method(self):
        self.tmp = tempfile.mkdtemp()
        self.exp = ReplayExporter(self.tmp)

    def test_file_created(self):
        store = make_store(2, 3)
        rd = self.exp.build_replay(store)
        path = self.exp.export_csv_frames(rd, 'test_tracks.csv')
        assert os.path.exists(path)

    def test_csv_has_header(self):
        store = make_store(2, 3)
        rd = self.exp.build_replay(store)
        path = self.exp.export_csv_frames(rd, 'test_tracks.csv')
        with open(path) as f:
            reader = csv.DictReader(f)
            assert 'mmsi' in reader.fieldnames
            assert 'latitude' in reader.fieldnames
            assert 'frame_timestamp' in reader.fieldnames

    def test_csv_row_count(self):
        store = make_store(2, 3, dt=1.0)
        rd = self.exp.build_replay(store, frame_interval=1.0)
        path = self.exp.export_csv_frames(rd, 'test_tracks.csv')
        with open(path) as f:
            rows = list(csv.DictReader(f))
        # 3帧 x 2船 = 6行
        assert len(rows) == rd.frame_count * 2

    def test_empty_replay_creates_file(self):
        rd = ReplayData()
        path = self.exp.export_csv_frames(rd, 'empty.csv')
        assert os.path.exists(path)


# ============================================================================
# 属性测试
# ============================================================================

@given(
    n_ships=st.integers(min_value=1, max_value=5),
    n_steps=st.integers(min_value=2, max_value=10),
    interval=st.floats(min_value=0.5, max_value=5.0,
                       allow_nan=False, allow_infinity=False),
)
@settings(max_examples=20)
def test_property_frame_has_ships(n_ships, n_steps, interval):
    """Property: 每帧都至少包含1艘船"""
    store = make_store(n_ships, n_steps, dt=interval)
    exp = ReplayExporter('/tmp')
    rd = exp.build_replay(store, frame_interval=interval)
    for frame in rd.frames:
        assert len(frame.ships) >= 1


@given(
    n_steps=st.integers(min_value=2, max_value=8),
)
@settings(max_examples=20)
def test_property_json_always_valid(n_steps):
    """Property: JSON导出始终产生合法JSON"""
    store = make_store(2, n_steps)
    with tempfile.TemporaryDirectory() as tmp:
        exp = ReplayExporter(tmp)
        rd = exp.build_replay(store)
        path = exp.export_json(rd, 'prop_test.json')
        with open(path) as f:
            data = json.load(f)
        assert isinstance(data['frames'], list)


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
