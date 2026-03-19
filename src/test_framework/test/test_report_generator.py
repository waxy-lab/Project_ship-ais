"""
报告生成单元测试
Requirements: 6.6
"""
import pytest
import os
import json
import tempfile

from test_framework.test_framework.metrics import (
    PerformanceMetrics, BatchTestReport, ScenarioResult
)
from test_framework.test_framework.report_generator import ReportGenerator
from test_framework.test_framework.metrics_calculator import compute_batch_report


def make_metrics(result=ScenarioResult.SUCCESS, scenario_type="head_on",
                 min_dcpa=1.0, course_changes=0, orig_dist=10.0,
                 actual_dist=10.5, total_dec=5, compliant_dec=5):
    m = PerformanceMetrics(
        scenario_id=f"test_{scenario_type}",
        scenario_type=scenario_type,
        result=result,
        min_dcpa_nm=min_dcpa,
        course_change_count=course_changes,
        original_distance_nm=orig_dist,
        actual_distance_nm=actual_dist,
        distance_increase_pct=(
            (actual_dist - orig_dist) / orig_dist * 100 if orig_dist > 0 else 0),
        total_decisions=total_dec,
        compliant_decisions=compliant_dec,
        colregs_compliance_rate=(
            compliant_dec / total_dec if total_dec > 0 else 1.0),
    )
    return m


@pytest.fixture
def tmp_dir():
    with tempfile.TemporaryDirectory() as d:
        yield d


@pytest.fixture
def sample_report():
    ml = [
        make_metrics(ScenarioResult.SUCCESS, "head_on", min_dcpa=0.8,
                     course_changes=2, total_dec=10, compliant_dec=10),
        make_metrics(ScenarioResult.SUCCESS, "crossing", min_dcpa=1.2,
                     course_changes=1, total_dec=5, compliant_dec=4),
        make_metrics(ScenarioResult.COLLISION, "head_on", min_dcpa=0.02,
                     course_changes=0, total_dec=3, compliant_dec=2),
    ]
    return compute_batch_report(ml, "test_batch")


class TestReportGeneratorInit:
    def test_creates_output_dir(self, tmp_dir):
        out = os.path.join(tmp_dir, "reports")
        ReportGenerator(output_dir=out)
        assert os.path.isdir(out)

    def test_default_output_dir(self, tmp_dir):
        gen = ReportGenerator(output_dir=tmp_dir)
        assert gen.output_dir == tmp_dir


class TestJsonReport:
    def test_generates_file(self, tmp_dir, sample_report):
        gen = ReportGenerator(output_dir=tmp_dir)
        path = gen.generate_json_report(sample_report, "test.json")
        assert os.path.isfile(path)

    def test_file_is_valid_json(self, tmp_dir, sample_report):
        gen = ReportGenerator(output_dir=tmp_dir)
        path = gen.generate_json_report(sample_report, "test.json")
        with open(path) as f:
            data = json.load(f)
        assert "report" in data
        assert "scenarios" in data
        assert "generated_at" in data

    def test_report_contains_success_rate(self, tmp_dir, sample_report):
        gen = ReportGenerator(output_dir=tmp_dir)
        path = gen.generate_json_report(sample_report, "test.json")
        with open(path) as f:
            data = json.load(f)
        assert "success_rate" in data["report"]
        assert abs(data["report"]["success_rate"] - 2/3) < 1e-3

    def test_scenarios_count_matches(self, tmp_dir, sample_report):
        gen = ReportGenerator(output_dir=tmp_dir)
        path = gen.generate_json_report(sample_report, "test.json")
        with open(path) as f:
            data = json.load(f)
        assert len(data["scenarios"]) == 3

    def test_default_filename_generated(self, tmp_dir, sample_report):
        gen = ReportGenerator(output_dir=tmp_dir)
        path = gen.generate_json_report(sample_report)
        assert path.endswith(".json")
        assert os.path.isfile(path)


class TestCsvReport:
    def test_generates_file(self, tmp_dir, sample_report):
        gen = ReportGenerator(output_dir=tmp_dir)
        path = gen.generate_csv_report(sample_report, "test.csv")
        assert os.path.isfile(path)

    def test_has_header_row(self, tmp_dir, sample_report):
        gen = ReportGenerator(output_dir=tmp_dir)
        path = gen.generate_csv_report(sample_report, "test.csv")
        with open(path) as f:
            first_line = f.readline()
        assert "scenario_id" in first_line

    def test_has_data_rows(self, tmp_dir, sample_report):
        gen = ReportGenerator(output_dir=tmp_dir)
        path = gen.generate_csv_report(sample_report, "test.csv")
        with open(path) as f:
            lines = f.readlines()
        assert len(lines) == 4  # header + 3 data rows

    def test_empty_report_csv(self, tmp_dir):
        empty = BatchTestReport(report_id="empty", total_scenarios=0)
        gen = ReportGenerator(output_dir=tmp_dir)
        path = gen.generate_csv_report(empty, "empty.csv")
        assert os.path.isfile(path)


class TestTextSummary:
    def test_generates_file(self, tmp_dir, sample_report):
        gen = ReportGenerator(output_dir=tmp_dir)
        path = gen.generate_text_summary(sample_report, "test.txt")
        assert os.path.isfile(path)

    def test_contains_success_rate(self, tmp_dir, sample_report):
        gen = ReportGenerator(output_dir=tmp_dir)
        path = gen.generate_text_summary(sample_report, "test.txt")
        with open(path) as f:
            content = f.read()
        assert "66.7%" in content or "成功率" in content

    def test_contains_all_requirements_sections(self, tmp_dir, sample_report):
        gen = ReportGenerator(output_dir=tmp_dir)
        path = gen.generate_text_summary(sample_report, "test.txt")
        with open(path) as f:
            content = f.read()
        assert "6.1" in content
        assert "6.2" in content
        assert "6.3" in content
        assert "6.4" in content
        assert "6.5" in content

    def test_contains_report_id(self, tmp_dir, sample_report):
        gen = ReportGenerator(output_dir=tmp_dir)
        path = gen.generate_text_summary(sample_report, "test.txt")
        with open(path) as f:
            content = f.read()
        assert "test_batch" in content


class TestGenerateAll:
    def test_generates_three_files(self, tmp_dir, sample_report):
        gen = ReportGenerator(output_dir=tmp_dir)
        files = gen.generate_all(sample_report, prefix="test_all")
        assert set(files.keys()) == {"json", "csv", "txt"}
        for path in files.values():
            assert os.path.isfile(path)

    def test_files_have_correct_extensions(self, tmp_dir, sample_report):
        gen = ReportGenerator(output_dir=tmp_dir)
        files = gen.generate_all(sample_report, prefix="test_ext")
        assert files["json"].endswith(".json")
        assert files["csv"].endswith(".csv")
        assert files["txt"].endswith(".txt")

    def test_default_prefix_works(self, tmp_dir, sample_report):
        gen = ReportGenerator(output_dir=tmp_dir)
        files = gen.generate_all(sample_report)
        assert len(files) == 3
        for path in files.values():
            assert os.path.isfile(path)


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
