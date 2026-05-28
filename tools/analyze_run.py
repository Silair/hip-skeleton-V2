#!/usr/bin/env python3
"""Analyze hs_exoskeleton_v2 runtime CSV logs and generate metrics + HTML charts.

No third-party dependencies are required. The script estimates a reference gait
phase from filtered gait-signal peak/trough events, compares AO phase/frequency
against that reference, and summarizes torque safety/timing metrics.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from statistics import mean
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

TAU = 2.0 * math.pi
RAD_TO_DEG = 180.0 / math.pi

DEFAULT_EVALUATION_THRESHOLDS: Dict[str, float] = {
    "min_reference_events": 4.0,
    "phase_rmse_percent": 5.0,
    "phase_abs_error_p95_percent": 10.0,
    "frequency_rmse_hz": 0.10,
    "ao_reconstruction_rmse_rad": 0.20,
    "convergence_time_s_at_5_percent": 4.0,
    "max_abs_torque_nm": 8.0,
    "max_torque_rate_nm_s": 80.0,
    "freeze_torque_violation_count": 0.0,
    "disallow_output_torque_violation_count": 0.0,
    "stationary_false_assist_s": 0.5,
    "simultaneous_high_torque_ratio": 0.02,
    "peak_torque_phase_mae_deg": 15.0,
}

EVALUATION_DISCLAIMER = (
    "本评价仅用于离线控制日志质量检查。PASS 表示当前日志中的控制指标满足配置阈值，"
    "不代表临床安全性、人体实验许可或硬件实际输出验证。"
)


def fnum(row: Dict[str, str], key: str, default: float = 0.0) -> float:
    value = row.get(key, "")
    if value == "" or value is None:
        return default
    try:
        return float(value)
    except ValueError:
        return default


def wrap_to_pi(angle: float) -> float:
    return (angle + math.pi) % TAU - math.pi


def percentile(values: Sequence[float], pct: float) -> Optional[float]:
    if not values:
        return None
    ordered = sorted(values)
    idx = (len(ordered) - 1) * pct / 100.0
    lo = int(math.floor(idx))
    hi = int(math.ceil(idx))
    if lo == hi:
        return ordered[lo]
    return ordered[lo] + (ordered[hi] - ordered[lo]) * (idx - lo)


def rmse(values: Sequence[float]) -> Optional[float]:
    if not values:
        return None
    return math.sqrt(sum(v * v for v in values) / len(values))



def _fmt_eval(value: object, digits: int = 4) -> str:
    if value is None:
        return "N/A"
    if isinstance(value, float):
        return f"{value:.{digits}f}"
    return str(value)


def _compare_metric(value: float, operator: str, threshold: float) -> bool:
    if operator == "<=":
        return value <= threshold
    if operator == ">=":
        return value >= threshold
    if operator == "==":
        return value == threshold
    raise ValueError(f"unsupported operator: {operator}")


def _make_check(
    metrics: Dict[str, object],
    key: str,
    label: str,
    operator: str,
    threshold: float,
    unit: str,
    required: bool = True,
    fail_on_threshold: bool = True,
) -> Dict[str, object]:
    value = metrics.get(key)
    if value is None:
        status = "FAIL" if required else "SKIP"
        passed = not required
        return {
            "key": key,
            "label": label,
            "value": None,
            "operator": operator,
            "threshold": threshold,
            "unit": unit,
            "required": required,
            "status": status,
            "passed": passed,
            "message": f"{label}: insufficient data",
        }

    numeric_value = float(value)
    threshold_passed = _compare_metric(numeric_value, operator, threshold)
    status = "PASS" if threshold_passed else ("FAIL" if fail_on_threshold else "WARNING")
    return {
        "key": key,
        "label": label,
        "value": numeric_value,
        "operator": operator,
        "threshold": threshold,
        "unit": unit,
        "required": required,
        "status": status,
        "passed": threshold_passed or not fail_on_threshold,
        "message": f"{label}: {_fmt_eval(numeric_value)} {operator} {_fmt_eval(threshold)} {unit}".strip(),
    }


def _group(label: str, checks: Sequence[Dict[str, object]]) -> Dict[str, object]:
    failed = [check for check in checks if check["status"] == "FAIL"]
    warnings = [check for check in checks if check["status"] == "WARNING"]
    skipped = [check for check in checks if check["status"] == "SKIP"]
    status = "FAIL" if failed else ("WARNING" if warnings else "PASS")
    return {
        "label": label,
        "status": status,
        "checks": list(checks),
        "failed_count": len(failed),
        "warning_count": len(warnings),
        "skipped_count": len(skipped),
    }


def evaluate_metrics(metrics: Dict[str, object], thresholds: Optional[Dict[str, float]] = None) -> Dict[str, object]:
    limits = dict(DEFAULT_EVALUATION_THRESHOLDS)
    if thresholds:
        limits.update({k: float(v) for k, v in thresholds.items() if v is not None})

    max_torque = float(metrics.get("max_abs_torque_nm") or 0.0)
    torque_phase_required = max_torque > 1e-6

    phase_checks = [
        _make_check(metrics, "reference_event_count", "参考相位事件数", ">=", limits["min_reference_events"], "events"),
        _make_check(metrics, "phase_rmse_percent", "相位 RMSE", "<=", limits["phase_rmse_percent"], "% gait"),
        _make_check(metrics, "phase_abs_error_p95_percent", "相位误差 P95", "<=", limits["phase_abs_error_p95_percent"], "% gait"),
        _make_check(metrics, "frequency_rmse_hz", "频率 RMSE", "<=", limits["frequency_rmse_hz"], "Hz"),
        _make_check(metrics, "ao_reconstruction_rmse_rad", "AO 重构 RMSE", "<=", limits["ao_reconstruction_rmse_rad"], "rad", required=False),
        _make_check(metrics, "convergence_time_s_at_5_percent", "5% 相位误差收敛时间", "<=", limits["convergence_time_s_at_5_percent"], "s", required=False),
    ]
    torque_checks = [
        _make_check(metrics, "max_abs_torque_nm", "最大绝对力矩", "<=", limits["max_abs_torque_nm"], "Nm"),
        _make_check(metrics, "max_torque_rate_nm_s", "最大力矩变化率", "<=", limits["max_torque_rate_nm_s"], "Nm/s"),
        _make_check(metrics, "simultaneous_high_torque_ratio", "双侧同时高力矩比例", "<=", limits["simultaneous_high_torque_ratio"], "ratio", required=False),
        _make_check(metrics, "peak_torque_phase_mae_deg", "峰值力矩相位 MAE", "<=", limits["peak_torque_phase_mae_deg"], "deg", required=torque_phase_required),
    ]
    safety_checks = [
        _make_check(metrics, "freeze_torque_violation_count", "冻结时非零力矩违规", "==", limits["freeze_torque_violation_count"], "frames"),
        _make_check(metrics, "disallow_output_torque_violation_count", "禁止输出时非零力矩违规", "==", limits["disallow_output_torque_violation_count"], "frames"),
        _make_check(metrics, "stationary_false_assist_s", "静止误助力时长", "<=", limits["stationary_false_assist_s"], "s"),
    ]

    groups = {
        "phase_tracking": _group("相位/频率跟踪", phase_checks),
        "torque_assist": _group("助力力矩效果", torque_checks),
        "safety_gating": _group("安全门控", safety_checks),
    }
    failed_checks = [check for group in groups.values() for check in group["checks"] if check["status"] == "FAIL"]
    status = "FAIL" if failed_checks else "PASS"
    return {
        "status": status,
        "passed": status == "PASS",
        "summary": "控制效果达标" if status == "PASS" else f"控制效果未达标：{len(failed_checks)} 项检查失败",
        "disclaimer": EVALUATION_DISCLAIMER,
        "thresholds": limits,
        "groups": groups,
        "failed_checks": failed_checks,
    }


def read_rows(path: Path) -> List[Dict[str, str]]:
    with path.open(newline="", encoding="utf-8-sig") as f:
        return list(csv.DictReader(f))


def detect_phase_events(rows: Sequence[Dict[str, str]], peak_min_spread_deg: float) -> List[Tuple[float, float, str]]:
    """Return (time_s, unwrapped_reference_phase_rad, event_type)."""
    events: List[Tuple[float, float, str]] = []
    previous_velocity: Optional[float] = None
    previous_type: Optional[str] = None
    previous_phase: Optional[float] = None

    for row in rows:
        t = fnum(row, "MonoTimeS")
        signed_velocity = fnum(row, "SignedPhaseVelocityDegS", fnum(row, "PhaseVelocityDegS"))
        spread = fnum(row, "SpreadDeg")
        if previous_velocity is None:
            previous_velocity = signed_velocity
            continue

        event_type: Optional[str] = None
        base_phase = 0.0
        if previous_velocity > 0.0 and signed_velocity <= 0.0 and spread >= peak_min_spread_deg:
            event_type = "positive_peak"
            base_phase = math.pi / 2.0
        elif previous_velocity < 0.0 and signed_velocity >= 0.0 and spread >= peak_min_spread_deg:
            event_type = "negative_peak"
            base_phase = 3.0 * math.pi / 2.0

        previous_velocity = signed_velocity
        if event_type is None or event_type == previous_type:
            continue

        phase = base_phase
        if previous_phase is not None:
            while phase <= previous_phase:
                phase += TAU
        events.append((t, phase, event_type))
        previous_phase = phase
        previous_type = event_type

    return events


def interpolate_reference(rows: Sequence[Dict[str, str]], events: Sequence[Tuple[float, float, str]]) -> Tuple[List[Optional[float]], List[Optional[float]]]:
    ref_phase: List[Optional[float]] = [None] * len(rows)
    ref_freq: List[Optional[float]] = [None] * len(rows)
    if len(events) < 2:
        return ref_phase, ref_freq

    event_index = 0
    for i, row in enumerate(rows):
        t = fnum(row, "MonoTimeS")
        while event_index + 1 < len(events) and t > events[event_index + 1][0]:
            event_index += 1
        if event_index + 1 >= len(events):
            continue
        t0, p0, _ = events[event_index]
        t1, p1, _ = events[event_index + 1]
        if not (t0 <= t <= t1) or t1 <= t0:
            continue
        ratio = (t - t0) / (t1 - t0)
        phase = p0 + (p1 - p0) * ratio
        ref_phase[i] = phase % TAU
        ref_freq[i] = (p1 - p0) / TAU / (t1 - t0)
    return ref_phase, ref_freq


def torque_rates(rows: Sequence[Dict[str, str]], key: str) -> List[float]:
    rates: List[float] = []
    for prev, curr in zip(rows, rows[1:]):
        dt = fnum(curr, "MonoTimeS") - fnum(prev, "MonoTimeS")
        if dt <= 1e-9:
            dt = fnum(curr, "DtS", 0.0)
        if dt <= 1e-9:
            continue
        rates.append(abs(fnum(curr, key) - fnum(prev, key)) / dt)
    return rates


def local_peak_phase_errors(rows: Sequence[Dict[str, str]], lead_angle_rad: float) -> Tuple[List[float], List[float]]:
    left_errors: List[float] = []
    right_errors: List[float] = []
    left_values = [abs(fnum(r, "LeftTorqueCmd")) for r in rows]
    right_values = [abs(fnum(r, "RightTorqueCmd")) for r in rows]
    left_threshold = max(left_values or [0.0]) * 0.25
    right_threshold = max(right_values or [0.0]) * 0.25
    left_target = (math.pi - lead_angle_rad) % TAU
    right_target = (-lead_angle_rad) % TAU

    for i in range(1, len(rows) - 1):
        phase = fnum(rows[i], "Phase")
        left = fnum(rows[i], "LeftTorqueCmd")
        right = fnum(rows[i], "RightTorqueCmd")
        if left > left_threshold and left >= fnum(rows[i - 1], "LeftTorqueCmd") and left >= fnum(rows[i + 1], "LeftTorqueCmd"):
            left_errors.append(abs(wrap_to_pi(phase - left_target)) * RAD_TO_DEG)
        if right > right_threshold and right >= fnum(rows[i - 1], "RightTorqueCmd") and right >= fnum(rows[i + 1], "RightTorqueCmd"):
            right_errors.append(abs(wrap_to_pi(phase - right_target)) * RAD_TO_DEG)
    return left_errors, right_errors


def compute_metrics(rows: Sequence[Dict[str, str]], peak_min_spread_deg: float = 18.0, lead_angle_rad: float = 0.20, torque_epsilon_nm: float = 1e-6) -> Tuple[Dict[str, object], List[Optional[float]], List[Optional[float]], List[Tuple[float, float, str]]]:
    events = detect_phase_events(rows, peak_min_spread_deg)
    ref_phase, ref_freq = interpolate_reference(rows, events)

    phase_errors = [wrap_to_pi(fnum(row, "Phase") - ref) for row, ref in zip(rows, ref_phase) if ref is not None]
    abs_phase_percent = [abs(e) / TAU * 100.0 for e in phase_errors]
    freq_errors = [fnum(row, "Frequency") - ref for row, ref in zip(rows, ref_freq) if ref is not None]
    reconstruction_errors = [fnum(row, "AoSignalErrorRad") for row in rows if "AoSignalErrorRad" in row and row.get("AoSignalErrorRad", "") != ""]

    left_torques = [fnum(r, "LeftTorqueCmd") for r in rows]
    right_torques = [fnum(r, "RightTorqueCmd") for r in rows]
    max_abs_torque = max([0.0] + [abs(v) for v in left_torques + right_torques])
    left_rates = torque_rates(rows, "LeftTorqueCmd")
    right_rates = torque_rates(rows, "RightTorqueCmd")
    all_rates = left_rates + right_rates

    freeze_violations = 0
    disallow_violations = 0
    stationary_false_assist_s = 0.0
    simultaneous_high = 0
    high_threshold = max_abs_torque * 0.25 if max_abs_torque > 0 else float("inf")

    for row in rows:
        left_abs = abs(fnum(row, "LeftTorqueCmd"))
        right_abs = abs(fnum(row, "RightTorqueCmd"))
        torque_active = left_abs > torque_epsilon_nm or right_abs > torque_epsilon_nm
        dt = fnum(row, "DtS", 0.0)
        if fnum(row, "FreezeRequested", 0.0) >= 0.5 and torque_active:
            freeze_violations += 1
        if fnum(row, "AllowOutput", 1.0) < 0.5 and torque_active:
            disallow_violations += 1
        if fnum(row, "StopProbability") >= 0.8 and torque_active:
            stationary_false_assist_s += dt
        if left_abs >= high_threshold and right_abs >= high_threshold:
            simultaneous_high += 1

    left_peak_errors, right_peak_errors = local_peak_phase_errors(rows, lead_angle_rad)
    peak_errors = left_peak_errors + right_peak_errors
    duration = 0.0
    if rows:
        duration = max(0.0, fnum(rows[-1], "MonoTimeS") - fnum(rows[0], "MonoTimeS"))

    convergence_time_s: Optional[float] = None
    if phase_errors:
        valid_pairs = [(fnum(row, "MonoTimeS"), abs(wrap_to_pi(fnum(row, "Phase") - ref)) / TAU * 100.0) for row, ref in zip(rows, ref_phase) if ref is not None]
        for t, err in valid_pairs:
            if err <= 5.0:
                convergence_time_s = t - valid_pairs[0][0]
                break

    metrics: Dict[str, object] = {
        "sample_count": len(rows),
        "duration_s": duration,
        "reference_event_count": len(events),
        "phase_rmse_rad": rmse(phase_errors),
        "phase_rmse_deg": None if rmse(phase_errors) is None else rmse(phase_errors) * RAD_TO_DEG,
        "phase_rmse_percent": None if rmse(phase_errors) is None else rmse(phase_errors) / TAU * 100.0,
        "phase_mae_percent": None if not abs_phase_percent else mean(abs_phase_percent),
        "phase_abs_error_p95_percent": percentile(abs_phase_percent, 95.0),
        "frequency_rmse_hz": rmse(freq_errors),
        "frequency_mae_hz": None if not freq_errors else mean(abs(v) for v in freq_errors),
        "ao_reconstruction_rmse_rad": rmse(reconstruction_errors),
        "convergence_time_s_at_5_percent": convergence_time_s,
        "max_abs_torque_nm": max_abs_torque,
        "max_torque_rate_nm_s": max([0.0] + all_rates),
        "torque_rate_p95_nm_s": percentile(all_rates, 95.0),
        "freeze_torque_violation_count": freeze_violations,
        "disallow_output_torque_violation_count": disallow_violations,
        "stationary_false_assist_s": stationary_false_assist_s,
        "simultaneous_high_torque_ratio": 0.0 if not rows else simultaneous_high / len(rows),
        "left_peak_torque_phase_mae_deg": None if not left_peak_errors else mean(left_peak_errors),
        "right_peak_torque_phase_mae_deg": None if not right_peak_errors else mean(right_peak_errors),
        "peak_torque_phase_mae_deg": None if not peak_errors else mean(peak_errors),
    }
    return metrics, ref_phase, ref_freq, events


def fmt(value: object, digits: int = 4) -> str:
    if value is None:
        return "N/A"
    if isinstance(value, float):
        return f"{value:.{digits}f}"
    return str(value)


def series_points(rows: Sequence[Dict[str, str]], key: str) -> List[Tuple[float, float]]:
    return [(fnum(r, "MonoTimeS"), fnum(r, key)) for r in rows]


def svg_chart(series: Sequence[Tuple[str, Sequence[Tuple[float, float]], str]], width: int = 960, height: int = 260) -> str:
    all_points = [p for _, pts, _ in series for p in pts]
    if not all_points:
        return "<svg></svg>"
    xs = [p[0] for p in all_points]
    ys = [p[1] for p in all_points]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    if max_x <= min_x:
        max_x = min_x + 1.0
    if max_y <= min_y:
        max_y = min_y + 1.0
    pad_l, pad_r, pad_t, pad_b = 56, 16, 18, 34
    plot_w = width - pad_l - pad_r
    plot_h = height - pad_t - pad_b

    def transform(point: Tuple[float, float]) -> Tuple[float, float]:
        x, y = point
        sx = pad_l + (x - min_x) / (max_x - min_x) * plot_w
        sy = pad_t + (max_y - y) / (max_y - min_y) * plot_h
        return sx, sy

    parts = [f'<svg viewBox="0 0 {width} {height}" width="100%" height="{height}" role="img">']
    parts.append(f'<rect x="0" y="0" width="{width}" height="{height}" fill="#fff"/>')
    parts.append(f'<line x1="{pad_l}" y1="{pad_t + plot_h}" x2="{pad_l + plot_w}" y2="{pad_t + plot_h}" stroke="#999"/>')
    parts.append(f'<line x1="{pad_l}" y1="{pad_t}" x2="{pad_l}" y2="{pad_t + plot_h}" stroke="#999"/>')
    parts.append(f'<text x="6" y="{pad_t + 12}" font-size="11" fill="#555">{fmt(max_y, 2)}</text>')
    parts.append(f'<text x="6" y="{pad_t + plot_h}" font-size="11" fill="#555">{fmt(min_y, 2)}</text>')
    parts.append(f'<text x="{pad_l}" y="{height - 8}" font-size="11" fill="#555">t={fmt(min_x, 2)}s</text>')
    parts.append(f'<text x="{width - 90}" y="{height - 8}" font-size="11" fill="#555">t={fmt(max_x, 2)}s</text>')
    legend_x = pad_l + 8
    for idx, (name, pts, color) in enumerate(series):
        path = " ".join(f"{x:.2f},{y:.2f}" for x, y in map(transform, pts))
        parts.append(f'<polyline fill="none" stroke="{color}" stroke-width="1.6" points="{path}"/>')
        parts.append(f'<text x="{legend_x}" y="{pad_t + 14 + idx * 16}" font-size="12" fill="{color}">{name}</text>')
    parts.append("</svg>")
    return "\n".join(parts)


def write_reports(rows: Sequence[Dict[str, str]], metrics: Dict[str, object], evaluation: Dict[str, object], ref_phase: Sequence[Optional[float]], ref_freq: Sequence[Optional[float]], events: Sequence[Tuple[float, float, str]], output_dir: Path, source_csv: Path) -> Dict[str, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    metrics_path = output_dir / "metrics.json"
    metrics_path.write_text(json.dumps(metrics, indent=2, ensure_ascii=False), encoding="utf-8")
    evaluation_path = output_dir / "evaluation.json"
    evaluation_path.write_text(json.dumps(evaluation, indent=2, ensure_ascii=False), encoding="utf-8")

    enriched_path = output_dir / "enriched_timeseries.csv"
    fieldnames = list(rows[0].keys()) if rows else []
    for extra in ["ReferencePhase", "ReferenceFrequency", "PhaseErrorDeg", "PhaseErrorPercent"]:
        if extra not in fieldnames:
            fieldnames.append(extra)
    with enriched_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row, rp, rf in zip(rows, ref_phase, ref_freq):
            out = dict(row)
            out["ReferencePhase"] = "" if rp is None else f"{rp:.10f}"
            out["ReferenceFrequency"] = "" if rf is None else f"{rf:.10f}"
            if rp is None:
                out["PhaseErrorDeg"] = ""
                out["PhaseErrorPercent"] = ""
            else:
                err = wrap_to_pi(fnum(row, "Phase") - rp)
                out["PhaseErrorDeg"] = f"{err * RAD_TO_DEG:.6f}"
                out["PhaseErrorPercent"] = f"{err / TAU * 100.0:.6f}"
            writer.writerow(out)

    phase_points = [(fnum(r, "MonoTimeS"), fnum(r, "Phase") * RAD_TO_DEG) for r in rows]
    ref_points = [(fnum(r, "MonoTimeS"), rp * RAD_TO_DEG) for r, rp in zip(rows, ref_phase) if rp is not None]
    freq_points = series_points(rows, "Frequency")
    ref_freq_points = [(fnum(r, "MonoTimeS"), rf) for r, rf in zip(rows, ref_freq) if rf is not None]
    torque_left = series_points(rows, "LeftTorqueCmd")
    torque_right = series_points(rows, "RightTorqueCmd")
    intent_motion = series_points(rows, "MotionConfidence")
    intent_stop = series_points(rows, "StopProbability")
    signal = series_points(rows, "FilteredPhaseSignalRad")

    rows_html = "\n".join(f"<tr><th>{k}</th><td>{fmt(v)}</td></tr>" for k, v in metrics.items())
    evaluation_groups_html_parts = []
    for group_key, group in evaluation["groups"].items():
        check_rows = "\n".join(
            f"<tr class='status-{check['status'].lower()}'>"
            f"<td>{check['label']}</td>"
            f"<td>{check['status']}</td>"
            f"<td>{fmt(check['value'])}</td>"
            f"<td>{check['operator']} {fmt(check['threshold'])} {check['unit']}</td>"
            f"</tr>"
            for check in group["checks"]
        )
        evaluation_groups_html_parts.append(
            f"<h3>{group['label']} — {group['status']}</h3>"
            f"<table><tr><th>指标</th><th>状态</th><th>实际值</th><th>阈值</th></tr>{check_rows}</table>"
        )
    evaluation_groups_html = "\n".join(evaluation_groups_html_parts)
    failed_html = "" if not evaluation["failed_checks"] else "<ul>" + "".join(f"<li>{check['message']}</li>" for check in evaluation["failed_checks"]) + "</ul>"
    events_html = "\n".join(f"<li>{fmt(t, 3)}s — {kind} — ref phase {fmt(phase * RAD_TO_DEG, 2)}°</li>" for t, phase, kind in events[:24])
    html = f"""<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8"/>
<title>hs_exoskeleton_v2 Run Analysis</title>
<style>
body {{ font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif; margin: 24px; color: #222; }}
section {{ margin: 28px 0; }}
table {{ border-collapse: collapse; }}
th, td {{ border: 1px solid #ddd; padding: 6px 10px; text-align: left; }}
th {{ background: #f6f8fa; }}
.card {{ border: 1px solid #ddd; border-radius: 8px; padding: 16px; margin: 14px 0; }}
code {{ background: #f6f8fa; padding: 2px 4px; border-radius: 4px; }}
.status-pass td {{ background: #f0fff4; }}
.status-fail td {{ background: #fff5f5; }}
.status-warning td {{ background: #fffaf0; }}
.status-skip td {{ background: #f6f8fa; color: #666; }}
.badge {{ display: inline-block; padding: 4px 10px; border-radius: 999px; font-weight: 700; }}
.badge-pass {{ background: #d3f9d8; color: #176a2c; }}
.badge-fail {{ background: #ffe3e3; color: #9b1c1c; }}
</style>
</head>
<body>
<h1>hs_exoskeleton_v2 运行分析报告</h1>
<p>Source CSV: <code>{source_csv}</code></p>
<p>Phase RMSE: <strong>{fmt(metrics.get('phase_rmse_percent'), 3)}% gait cycle</strong>, Frequency RMSE: <strong>{fmt(metrics.get('frequency_rmse_hz'), 4)} Hz</strong></p>
<section class="card"><h2>控制效果判定</h2>
<p><span class="badge badge-{evaluation['status'].lower()}">{evaluation['status']}</span> {evaluation['summary']}</p>
<p><strong>说明：</strong>{evaluation['disclaimer']}</p>
{failed_html}
{evaluation_groups_html}
</section>
<section class="card"><h2>指标汇总</h2><table>{rows_html}</table></section>
<section class="card"><h2>AO 相位 vs 参考相位</h2>{svg_chart([('AO phase deg', phase_points, '#1f77b4'), ('reference phase deg', ref_points, '#ff7f0e')])}</section>
<section class="card"><h2>频率追踪</h2>{svg_chart([('AO frequency Hz', freq_points, '#2ca02c'), ('reference frequency Hz', ref_freq_points, '#d62728')])}</section>
<section class="card"><h2>力矩输出</h2>{svg_chart([('left torque Nm', torque_left, '#9467bd'), ('right torque Nm', torque_right, '#8c564b')])}</section>
<section class="card"><h2>意图概率</h2>{svg_chart([('motion confidence', intent_motion, '#17becf'), ('stop probability', intent_stop, '#e377c2')])}</section>
<section class="card"><h2>滤波步态信号</h2>{svg_chart([('filtered phase signal rad', signal, '#7f7f7f')])}</section>
<section class="card"><h2>参考相位事件（前 24 个）</h2><ol>{events_html}</ol></section>
<p>Enriched CSV: <code>{enriched_path.name}</code>, metrics JSON: <code>{metrics_path.name}</code>, evaluation JSON: <code>{evaluation_path.name}</code></p>
</body>
</html>
"""
    html_path = output_dir / "index.html"
    html_path.write_text(html, encoding="utf-8")
    return {"metrics": metrics_path, "evaluation": evaluation_path, "html": html_path, "enriched_csv": enriched_path}


def analyze_log(log_path: Path, output_dir: Optional[Path] = None, peak_min_spread_deg: float = 18.0, lead_angle_rad: float = 0.20, evaluation_thresholds: Optional[Dict[str, float]] = None) -> Dict[str, object]:
    rows = read_rows(log_path)
    metrics, ref_phase, ref_freq, events = compute_metrics(rows, peak_min_spread_deg=peak_min_spread_deg, lead_angle_rad=lead_angle_rad)
    evaluation = evaluate_metrics(metrics, thresholds=evaluation_thresholds)
    if output_dir is None:
        output_dir = log_path.with_suffix("")
        output_dir = output_dir.parent / f"{output_dir.name}_analysis"
    paths = write_reports(rows, metrics, evaluation, ref_phase, ref_freq, events, output_dir, log_path)
    return {"metrics": metrics, "evaluation": evaluation, "paths": paths, "event_count": len(events)}


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Analyze hs_exoskeleton_v2 runtime CSV and generate HTML charts.")
    parser.add_argument("csv", type=Path, help="Runtime CSV generated by ExoLogger")
    parser.add_argument("--output-dir", type=Path, default=None, help="Report directory; default: <csv>_analysis")
    parser.add_argument("--peak-min-spread-deg", type=float, default=18.0, help="Minimum spread for reference phase peak/trough events")
    parser.add_argument("--lead-angle-rad", type=float, default=0.20, help="TorqueProfile lead angle used for peak torque timing metrics")
    parser.add_argument("--min-reference-events", type=float, default=None, help="Minimum reference phase events required for PASS")
    parser.add_argument("--phase-rmse-threshold-percent", type=float, default=None, help="Maximum phase RMSE in percent of gait cycle")
    parser.add_argument("--phase-p95-threshold-percent", type=float, default=None, help="Maximum phase P95 absolute error in percent of gait cycle")
    parser.add_argument("--frequency-rmse-threshold-hz", type=float, default=None, help="Maximum frequency RMSE in Hz")
    parser.add_argument("--max-torque-nm", type=float, default=None, help="Maximum allowed absolute commanded torque in Nm")
    parser.add_argument("--max-torque-rate-nm-s", type=float, default=None, help="Maximum allowed torque rate in Nm/s")
    parser.add_argument("--stationary-false-assist-threshold-s", type=float, default=None, help="Maximum allowed assist duration while stop probability is high")
    parser.add_argument("--peak-torque-phase-threshold-deg", type=float, default=None, help="Maximum peak torque phase MAE in degrees")
    parser.add_argument("--fail-on-evaluation", action="store_true", help="Exit 2 when evaluation status is FAIL")
    args = parser.parse_args(argv)

    threshold_overrides = {
        "min_reference_events": args.min_reference_events,
        "phase_rmse_percent": args.phase_rmse_threshold_percent,
        "phase_abs_error_p95_percent": args.phase_p95_threshold_percent,
        "frequency_rmse_hz": args.frequency_rmse_threshold_hz,
        "max_abs_torque_nm": args.max_torque_nm,
        "max_torque_rate_nm_s": args.max_torque_rate_nm_s,
        "stationary_false_assist_s": args.stationary_false_assist_threshold_s,
        "peak_torque_phase_mae_deg": args.peak_torque_phase_threshold_deg,
    }
    threshold_overrides = {k: v for k, v in threshold_overrides.items() if v is not None}
    result = analyze_log(args.csv, output_dir=args.output_dir, peak_min_spread_deg=args.peak_min_spread_deg, lead_angle_rad=args.lead_angle_rad, evaluation_thresholds=threshold_overrides)
    metrics = result["metrics"]
    print("Analysis complete")
    print(f"  phase_rmse_percent: {fmt(metrics.get('phase_rmse_percent'), 3)}")
    print(f"  frequency_rmse_hz:  {fmt(metrics.get('frequency_rmse_hz'), 4)}")
    print(f"  max_abs_torque_nm:  {fmt(metrics.get('max_abs_torque_nm'), 3)}")
    print(f"  evaluation_status:  {result['evaluation']['status']}")
    print(f"  html:               {result['paths']['html']}")
    print(f"  metrics:            {result['paths']['metrics']}")
    print(f"  evaluation:         {result['paths']['evaluation']}")
    if args.fail_on_evaluation and result['evaluation']['status'] == "FAIL":
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
