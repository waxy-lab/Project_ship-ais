"""
报告生成器：批量测试报告生成
Requirements: 6.6
"""
import json
import csv
import os
from datetime import datetime
from typing import List, Optional

from .metrics import PerformanceMetrics, BatchTestReport, ScenarioResult


class ReportGenerator:
    """
    性能报告生成器
    支持 JSON、CSV、文本摘要三种格式
    Requirements: 6.6
    """

    def __init__(self, output_dir: str = "."):
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)

    def generate_json_report(self, report: BatchTestReport,
                              filename: Optional[str] = None) -> str:
        """生成 JSON 格式报告 Requirements: 6.6"""
        if filename is None:
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"report_{report.report_id}_{ts}.json"
        filepath = os.path.join(self.output_dir, filename)
        data = {
            "report": report.to_dict(),
            "scenarios": [m.to_dict() for m in report.scenario_metrics],
            "generated_at": datetime.now().isoformat(),
        }
        with open(filepath, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        return filepath

    def generate_csv_report(self, report: BatchTestReport,
                             filename: Optional[str] = None) -> str:
        """生成 CSV 格式报告 Requirements: 6.6"""
        if filename is None:
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"report_{report.report_id}_{ts}.csv"
        filepath = os.path.join(self.output_dir, filename)
        if not report.scenario_metrics:
            with open(filepath, 'w', newline='', encoding='utf-8') as f:
                f.write("no data\n")
            return filepath
        rows = [m.to_dict() for m in report.scenario_metrics]
        # 移除复杂字段
        skip_keys = {'notes'}
        fieldnames = [k for k in rows[0].keys() if k not in skip_keys]
        with open(filepath, 'w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames,
                                    extrasaction='ignore')
            writer.writeheader()
            writer.writerows(rows)
        return filepath

    def generate_text_summary(self, report: BatchTestReport,
                               filename: Optional[str] = None) -> str:
        """生成文本摘要报告 Requirements: 6.6"""
        if filename is None:
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"summary_{report.report_id}_{ts}.txt"
        filepath = os.path.join(self.output_dir, filename)
        lines = [
            "=" * 60,
            f"  避碰算法性能评估报告",
            f"  报告 ID: {report.report_id}",
            f"  生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
            "=" * 60,
            "",
            f"【场景统计】",
            f"  总场景数:    {report.total_scenarios}",
            f"  成功场景:    {report.success_count}",
            f"  碰撞场景:    {report.collision_count}",
            f"  险情次数:    {report.near_miss_count}",
            "",
            f"【Req 6.1】避碰成功率",
            f"  总体成功率:  {report.success_rate*100:.1f}%",
        ]
        if report.success_rate_by_type:
            lines.append("  按场景类型:")
            for t, r in sorted(report.success_rate_by_type.items()):
                lines.append(f"    {t:15s}: {r*100:.1f}%")
        min_dcpa = report.overall_min_dcpa_nm
        min_dcpa_str = f"{min_dcpa:.3f} nm" if min_dcpa != float('inf') else "N/A"
        lines += [
            "",
            f"【Req 6.2】避让距离",
            f"  平均最小DCPA: {report.avg_min_dcpa_nm:.3f} nm",
            f"  全局最小DCPA: {min_dcpa_str}",
            "",
            f"【Req 6.3】航向改变",
            f"  平均改变次数: {report.avg_course_change_count:.1f} 次",
            f"  平均改变角度: {report.avg_course_change_deg:.1f} 度",
            "",
            f"【Req 6.4】航程与时间",
            f"  平均航程增加: {report.avg_distance_increase_pct:.1f}%",
            f"  平均时间延误: {report.avg_time_delay_sec:.1f} 秒",
            "",
            f"【Req 6.5】COLREGS 遵守率",
            f"  平均遵守率:   {report.avg_colregs_compliance_rate*100:.1f}%",
            "",
            "=" * 60,
        ]
        content = "\n".join(lines)
        with open(filepath, 'w', encoding='utf-8') as f:
            f.write(content)
        return filepath

    def generate_all(self, report: BatchTestReport,
                     prefix: Optional[str] = None) -> dict:
        """生成所有格式报告，返回文件路径字典"""
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        p = prefix or f"report_{report.report_id}_{ts}"
        return {
            "json": self.generate_json_report(report, f"{p}.json"),
            "csv": self.generate_csv_report(report, f"{p}.csv"),
            "txt": self.generate_text_summary(report, f"{p}.txt"),
        }
