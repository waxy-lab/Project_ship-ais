"""
数据记录模块单元测试
Requirements: 9.1-9.4
Property 15: 数据记录完整性
Property 16: 决策时间戳一致性
Property 17: 导出格式可解析性
"""
import json, csv, os, time, tempfile, sys, pytest
from hypothesis import given, settings, strategies as st
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from data_logger.data_models import (
    AisRecord, DecisionRecord, RiskRecord, DataStore, DataExporter
)

def make_ais(mmsi=111, lat=30.0, lon=120.0, hdg=90.0, sog=12.0, ts=None):
    return AisRecord(ts or time.time(), mmsi, lat, lon, hdg, sog, 0.0)

def make_decision(mmsi=111, dtype="TURN_RIGHT", ts=None):
    return DecisionRecord(time.time() if ts is None else ts, mmsi, dtype, 100.0, 12.0, "avoidance")

def make_risk(own=111, target=222, dcpa=0.5, tcpa=120.0, cri=0.6, ts=None):
    return RiskRecord(ts or time.time(), own, target, dcpa, tcpa, cri, "warning")

@pytest.fixture
def store(): return DataStore()

@pytest.fixture
def tmp_dir():
    with tempfile.TemporaryDirectory() as d: yield d

class TestAisRecord:
    def test_to_dict_fields(self):
        d = make_ais().to_dict()
        for f in ['timestamp','mmsi','latitude','longitude','heading','sog','rot']:
            assert f in d
    def test_values_preserved(self):
        d = make_ais(mmsi=999, lat=31.5, hdg=45.0).to_dict()
        assert d['mmsi']==999 and d['latitude']==31.5 and d['heading']==45.0

class TestDecisionRecord:
    def test_to_dict_fields(self):
        d = make_decision().to_dict()
        for f in ['timestamp','own_mmsi','decision_type','target_heading','target_speed','reason']:
            assert f in d
    def test_optional_none(self):
        r = DecisionRecord(time.time(), 111, "MAINTAIN", None, None, "safe")
        d = r.to_dict()
        assert d['target_heading'] is None and d['target_speed'] is None

class TestRiskRecord:
    def test_to_dict_fields(self):
        d = make_risk().to_dict()
        for f in ['timestamp','own_mmsi','target_mmsi','dcpa','tcpa','cri','risk_level']:
            assert f in d

class TestDataStoreAis:
    def test_add_single(self, store):
        store.add_ais(make_ais()); assert store.ais_count==1
    def test_add_multiple(self, store):
        for i in range(10): store.add_ais(make_ais(mmsi=100+i))
        assert store.ais_count==10
    def test_get_by_mmsi(self, store):
        store.add_ais(make_ais(111)); store.add_ais(make_ais(222)); store.add_ais(make_ais(111))
        assert len(store.get_ais_by_mmsi(111))==2
        assert len(store.get_ais_by_mmsi(999))==0
    def test_get_returns_copy(self, store):
        store.add_ais(make_ais()); store.get_ais_records().clear(); assert store.ais_count==1
    def test_clear(self, store):
        store.add_ais(make_ais()); store.clear(); assert store.ais_count==0

@given(n=st.integers(min_value=1, max_value=50))
@settings(max_examples=20)
def test_property15_ais_completeness(n):
    """Property 15: 记录数量与写入数量完全一致"""
    store = DataStore()
    for i in range(n): store.add_ais(make_ais(mmsi=100+i))
    assert store.ais_count==n and len(store.get_ais_records())==n

class TestDecisionStore:
    def test_add(self, store): store.add_decision(make_decision()); assert store.decision_count==1
    def test_types_preserved(self, store):
        store.add_decision(make_decision(dtype="TURN_RIGHT"))
        store.add_decision(make_decision(dtype="SLOW_DOWN"))
        types = {r.decision_type for r in store.get_decision_records()}
        assert "TURN_RIGHT" in types and "SLOW_DOWN" in types

@given(timestamps=st.lists(
    st.floats(min_value=0.0, max_value=1e9, allow_nan=False, allow_infinity=False),
    min_size=1, max_size=20))
@settings(max_examples=20)
def test_property16_decision_timestamp_consistency(timestamps):
    """Property 16: 按顺序插入的时间戳保持一致"""
    store = DataStore()
    sorted_ts = sorted(timestamps)
    for ts in sorted_ts: store.add_decision(make_decision(ts=ts))
    assert [r.timestamp for r in store.get_decision_records()] == sorted_ts

class TestRiskStore:
    def test_add(self, store): store.add_risk(make_risk()); assert store.risk_count==1
    def test_values(self, store):
        store.add_risk(make_risk(dcpa=0.3, tcpa=60.0, cri=0.8))
        r = store.get_risk_records()[0]
        assert abs(r.dcpa-0.3)<1e-9 and abs(r.tcpa-60.0)<1e-9

class TestDataExporter:
    def test_creates_dir(self, tmp_dir):
        out = os.path.join(tmp_dir, "sub")
        DataExporter(output_dir=out); assert os.path.isdir(out)
    def test_ais_csv_parseable(self, tmp_dir):
        exp = DataExporter(output_dir=tmp_dir)
        path = exp.export_ais_csv([make_ais(mmsi=111, lat=30.1)], "ais.csv")
        rows = list(csv.DictReader(open(path)))
        assert len(rows)==1 and int(rows[0]['mmsi'])==111
    def test_ais_csv_header(self, tmp_dir):
        exp = DataExporter(output_dir=tmp_dir)
        path = exp.export_ais_csv([make_ais()], "ais.csv")
        hdr = open(path).readline()
        assert 'mmsi' in hdr and 'latitude' in hdr
    def test_decisions_csv_parseable(self, tmp_dir):
        exp = DataExporter(output_dir=tmp_dir)
        path = exp.export_decisions_csv([make_decision(dtype="TURN_RIGHT")], "dec.csv")
        rows = list(csv.DictReader(open(path)))
        assert rows[0]['decision_type']=='TURN_RIGHT'
    def test_risks_csv_parseable(self, tmp_dir):
        exp = DataExporter(output_dir=tmp_dir)
        path = exp.export_risks_csv([make_risk(dcpa=0.5)], "risk.csv")
        rows = list(csv.DictReader(open(path)))
        assert abs(float(rows[0]['dcpa'])-0.5)<1e-6
    def test_json_all_valid(self, tmp_dir):
        store = DataStore()
        store.add_ais(make_ais()); store.add_decision(make_decision()); store.add_risk(make_risk())
        exp = DataExporter(output_dir=tmp_dir)
        data = json.load(open(exp.export_all_json(store, "all.json")))
        assert data['summary']['ais_count']==1 and 'ais_records' in data
    def test_empty_csv(self, tmp_dir):
        path = DataExporter(output_dir=tmp_dir).export_ais_csv([], "empty.csv")
        assert os.path.isfile(path)

@given(n=st.integers(min_value=1, max_value=30))
@settings(max_examples=20)
def test_property17_json_roundtrip(n):
    """Property 17: JSON导出后数据点数一致"""
    with tempfile.TemporaryDirectory() as d:
        store = DataStore()
        for i in range(n):
            store.add_ais(make_ais(mmsi=100+i))
            store.add_decision(make_decision(mmsi=100+i))
            store.add_risk(make_risk(own=100+i, target=200+i))
        data = json.load(open(DataExporter(output_dir=d).export_all_json(store)))
        assert data['summary']['ais_count']==n
        assert data['summary']['decision_count']==n
        assert len(data['ais_records'])==n

@given(n=st.integers(min_value=1, max_value=30))
@settings(max_examples=20)
def test_property17_csv_roundtrip(n):
    """Property 17: CSV行数 = 记录数 + header"""
    with tempfile.TemporaryDirectory() as d:
        records = [make_ais(mmsi=100+i) for i in range(n)]
        path = DataExporter(output_dir=d).export_ais_csv(records)
        assert len(open(path).readlines()) == n+1

if __name__ == '__main__':
    pytest.main([__file__, '-v'])
