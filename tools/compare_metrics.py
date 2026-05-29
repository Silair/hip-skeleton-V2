#!/usr/bin/env python3
"""Compare two hs_exoskeleton_v2 analysis metrics.json files.

This is meant for controller A/B checks: run the same replay curve before and
after an AO/control change, then compare tracking error and cadence adaptation
speed with one small, reviewable summary.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Dict, List, Optional, Sequence


METRIC_SPECS = [
    ("phase_rmse_percent", "Phase RMSE", "lower_is_better", "% gait"),
    ("phase_abs_error_p95_percent", "Phase error P95", "lower_is_better", "% gait"),
    ("frequency_rmse_hz", "Frequency RMSE", "lower_is_better", "Hz"),
    ("frequency_mae_hz", "Frequency MAE", "lower_is_better", "Hz"),
    ("frequency_adaptation_time_mean_s", "Frequency relock mean", "lower_is_better", "s"),
    ("frequency_adaptation_time_max_s", "Frequency relock max", "lower_is_better", "s"),
    ("phase_adaptation_time_mean_s", "Phase relock mean", "lower_is_better", "s"),
    ("phase_adaptation_time_max_s", "Phase relock max", "lower_is_better", "s"),
    ("combined_adaptation_time_mean_s", "Combined relock mean", "lower_is_better", "s"),
    ("combined_adaptation_time_max_s", "Combined relock max", "lower_is_better", "s"),
    ("max_torque_rate_nm_s", "Max torque rate", "lower_is_better", "Nm/s"),
    ("peak_torque_phase_mae_deg", "Peak torque phase MAE", "lower_is_better", "deg"),
]


def _as_float(metrics: Dict[str, object], key: str) -> Optional[float]:
    value = metrics.get(key)
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def compare_metric_sets(before: Dict[str, object], after: Dict[str, object]) -> Dict[str, object]:
    rows: List[Dict[str, object]] = []
    improved = 0
    worsened = 0
    comparable = 0

    for key, label, direction, unit in METRIC_SPECS:
        before_value = _as_float(before, key)
        after_value = _as_float(after, key)
        delta: Optional[float] = None
        relative: Optional[float] = None
        verdict = "SKIP"
        if before_value is not None and after_value is not None:
            delta = after_value - before_value
            if abs(before_value) > 1e-12:
                relative = delta / abs(before_value) * 100.0
            comparable += 1
            if abs(delta) <= 1e-12:
                verdict = "UNCHANGED"
            elif direction == "lower_is_better":
                verdict = "IMPROVED" if delta < 0.0 else "WORSE"
            else:
                verdict = "IMPROVED" if delta > 0.0 else "WORSE"
            improved += 1 if verdict == "IMPROVED" else 0
            worsened += 1 if verdict == "WORSE" else 0

        rows.append(
            {
                "key": key,
                "label": label,
                "unit": unit,
                "direction": direction,
                "before": before_value,
                "after": after_value,
                "delta": delta,
                "relative_change_percent": relative,
                "verdict": verdict,
            }
        )

    if comparable == 0:
        status = "NO_COMPARABLE_METRICS"
    elif worsened == 0 and improved > 0:
        status = "IMPROVED"
    elif improved == 0 and worsened > 0:
        status = "WORSE"
    elif worsened > 0 and improved > 0:
        status = "MIXED"
    else:
        status = "UNCHANGED"

    return {
        "status": status,
        "comparable_metric_count": comparable,
        "improved_metric_count": improved,
        "worsened_metric_count": worsened,
        "metrics": rows,
    }


def format_markdown(summary: Dict[str, object]) -> str:
    lines = [
        "# Metrics comparison",
        "",
        f"Status: **{summary['status']}**",
        "",
        "| Metric | Before | After | Δ | Δ% | Verdict |",
        "| --- | ---: | ---: | ---: | ---: | --- |",
    ]
    for row in summary["metrics"]:
        before = "N/A" if row["before"] is None else f"{row['before']:.4f}"
        after = "N/A" if row["after"] is None else f"{row['after']:.4f}"
        delta = "N/A" if row["delta"] is None else f"{row['delta']:+.4f}"
        relative = "N/A" if row["relative_change_percent"] is None else f"{row['relative_change_percent']:+.1f}%"
        lines.append(f"| {row['label']} ({row['unit']}) | {before} | {after} | {delta} | {relative} | {row['verdict']} |")
    return "\n".join(lines) + "\n"


def read_metrics(path: Path) -> Dict[str, object]:
    return json.loads(path.read_text(encoding="utf-8"))


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Compare two analyze_run.py metrics.json files.")
    parser.add_argument("before", type=Path, help="Baseline metrics.json")
    parser.add_argument("after", type=Path, help="Candidate metrics.json")
    parser.add_argument("--json-output", type=Path, default=None, help="Optional path to write machine-readable comparison JSON")
    parser.add_argument("--markdown-output", type=Path, default=None, help="Optional path to write Markdown comparison")
    args = parser.parse_args(argv)

    summary = compare_metric_sets(read_metrics(args.before), read_metrics(args.after))
    markdown = format_markdown(summary)
    print(markdown, end="")

    if args.json_output:
        args.json_output.write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    if args.markdown_output:
        args.markdown_output.write_text(markdown, encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
