#!/usr/bin/env python3
"""Diagnose fixed-link low-torque hardware trials from hs_exoskeleton_v2 CSV logs."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from statistics import mean
from typing import Dict, Iterable, List, Optional, Sequence


ASSIST_STATE_NAMES = {
    0: "Transparent",
    1: "Tracking",
    2: "Ramp",
    3: "Active",
    4: "Stopping",
    5: "Frozen",
    6: "Fault",
}

SIDE_PREFIX = {
    "left": "Left",
    "right": "Right",
}


def fnum(row: Dict[str, str], key: str, default: float = 0.0) -> float:
    try:
        return float(row.get(key, default))
    except (TypeError, ValueError):
        return default


def load_rows(path: Path) -> List[Dict[str, str]]:
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def span(values: Sequence[float]) -> float:
    if not values:
        return 0.0
    return max(values) - min(values)


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


def position_assessment(delta_rad: float) -> str:
    if delta_rad > 0.05:
        return "FAIL"
    if delta_rad > 0.03:
        return "WARN"
    return "PASS"


def velocity_assessment(max_abs_rad_s: float) -> str:
    if max_abs_rad_s > 0.1:
        return "WARN"
    return "PASS"


def overall_status(statuses: Iterable[str]) -> str:
    values = list(statuses)
    if "FAIL" in values:
        return "FAIL"
    if "WARN" in values:
        return "WARN"
    return "PASS"


def summarize_fixed_link(rows: Sequence[Dict[str, str]], *, side: str) -> Dict[str, object]:
    prefix = SIDE_PREFIX[side]
    raw_pos = [fnum(row, f"{prefix}RawMotorPos") for row in rows]
    raw_vel_abs = [abs(fnum(row, f"{prefix}RawMotorVel")) for row in rows]
    joint_pos = [fnum(row, f"{prefix}JointPos") for row in rows]
    joint_vel_abs = [abs(fnum(row, f"{prefix}JointVel")) for row in rows]
    torque_abs = [abs(fnum(row, f"{prefix}TorqueCmd")) for row in rows]
    allow_output = [fnum(row, "AllowOutput") for row in rows]

    raw_delta = span(raw_pos)
    raw_vel_max = max(raw_vel_abs, default=0.0)
    pos_status = position_assessment(raw_delta)
    vel_status = velocity_assessment(raw_vel_max)

    findings: List[str] = []
    if raw_delta > 0.05:
        findings.append("raw_motor_position_drift_over_0p05_rad")
    elif raw_delta > 0.03:
        findings.append("raw_motor_position_drift_over_0p03_rad")
    elif raw_delta > 0.01:
        findings.append("raw_motor_position_drift_over_0p01_rad")

    if raw_vel_max > 0.1:
        findings.append("raw_motor_velocity_spike_over_0p1_rad_s")
    elif raw_vel_max > 0.02:
        findings.append("raw_motor_velocity_over_0p02_rad_s")

    zero_torque_motion_count = sum(
        1
        for row in rows
        if fnum(row, "AllowOutput") < 0.5
        and abs(fnum(row, f"{prefix}TorqueCmd")) < 1e-6
        and abs(fnum(row, f"{prefix}RawMotorVel")) > 0.1
    )
    if zero_torque_motion_count:
        findings.append("raw_motor_moving_with_zero_output")

    return {
        "side": side,
        "sample_count": len(rows),
        "duration_s": 0.0 if not rows else max(0.0, fnum(rows[-1], "MonoTimeS") - fnum(rows[0], "MonoTimeS")),
        "raw_motor_pos_delta_rad": raw_delta,
        "raw_motor_pos_delta_deg": raw_delta * 57.2957795130823,
        "max_abs_raw_motor_vel_rad_s": raw_vel_max,
        "raw_motor_vel_p95_rad_s": percentile(raw_vel_abs, 95.0),
        "joint_pos_delta_rad": span(joint_pos),
        "max_abs_joint_vel_rad_s": max(joint_vel_abs, default=0.0),
        "max_abs_torque_cmd_nm": max(torque_abs, default=0.0),
        "allow_output_ratio": 0.0 if not allow_output else mean(1.0 if value >= 0.5 else 0.0 for value in allow_output),
        "zero_torque_motion_count": zero_torque_motion_count,
        "position_assessment": pos_status,
        "velocity_assessment": vel_status,
        "status": overall_status([pos_status, vel_status]),
        "findings": findings,
    }


def summarize_stop_response(rows: Sequence[Dict[str, str]]) -> Dict[str, object]:
    active_or_ramp_after_stop_intent = [
        row
        for row in rows
        if fnum(row, "StopProbability") >= 0.8 and int(fnum(row, "AssistState", -1)) in (2, 3)
    ]
    stopping_rows = [row for row in rows if int(fnum(row, "AssistState", -1)) == 4]
    output_disabled_rows = [row for row in rows if fnum(row, "AllowOutput") < 0.5]

    moving_after_output_disabled = [
        row
        for row in output_disabled_rows
        if max(abs(fnum(row, "LeftRawMotorVel")), abs(fnum(row, "RightRawMotorVel"))) > 0.1
        and max(abs(fnum(row, "LeftTorqueCmd")), abs(fnum(row, "RightTorqueCmd"))) < 1e-6
    ]
    torque_while_disabled = [
        row
        for row in output_disabled_rows
        if max(abs(fnum(row, "LeftTorqueCmd")), abs(fnum(row, "RightTorqueCmd"))) > 1e-6
    ]

    findings: List[str] = []
    if active_or_ramp_after_stop_intent:
        findings.append("active_or_ramp_after_stop_intent")
    if moving_after_output_disabled:
        findings.append("raw_motor_moving_after_output_disabled")
    if torque_while_disabled:
        findings.append("torque_command_nonzero_while_output_disabled")
    if not stopping_rows:
        findings.append("stopping_state_not_observed")

    status = "FAIL" if active_or_ramp_after_stop_intent or torque_while_disabled else "PASS"
    if status == "PASS" and (moving_after_output_disabled or not stopping_rows):
        status = "WARN"

    final = rows[-1] if rows else {}
    return {
        "status": status,
        "sample_count": len(rows),
        "stopping_sample_count": len(stopping_rows),
        "active_or_ramp_after_stop_intent_count": len(active_or_ramp_after_stop_intent),
        "moving_after_output_disabled_count": len(moving_after_output_disabled),
        "torque_while_output_disabled_count": len(torque_while_disabled),
        "final_assist_state": ASSIST_STATE_NAMES.get(int(fnum(final, "AssistState", -1)), str(int(fnum(final, "AssistState", -1)))),
        "final_allow_output": int(fnum(final, "AllowOutput", 0.0) >= 0.5),
        "final_stop_probability": fnum(final, "StopProbability"),
        "final_motion_confidence": fnum(final, "MotionConfidence"),
        "findings": findings,
    }


def compare_zeroing(
    zeroed_rows: Sequence[Dict[str, str]], unzeroed_rows: Sequence[Dict[str, str]]
) -> Dict[str, object]:
    def series(rows: Sequence[Dict[str, str]], key: str) -> List[float]:
        return [fnum(row, key) for row in rows]

    zeroed_spread = series(zeroed_rows, "SpreadDeg")
    unzeroed_spread = series(unzeroed_rows, "SpreadDeg")
    zeroed_motion = series(zeroed_rows, "MotionConfidence")
    unzeroed_motion = series(unzeroed_rows, "MotionConfidence")
    zeroed_stop = series(zeroed_rows, "StopProbability")
    unzeroed_stop = series(unzeroed_rows, "StopProbability")

    initial_spread_delta = (unzeroed_spread[0] if unzeroed_spread else 0.0) - (zeroed_spread[0] if zeroed_spread else 0.0)
    mean_motion_delta = (mean(unzeroed_motion) if unzeroed_motion else 0.0) - (mean(zeroed_motion) if zeroed_motion else 0.0)
    mean_stop_delta = (mean(unzeroed_stop) if unzeroed_stop else 0.0) - (mean(zeroed_stop) if zeroed_stop else 0.0)

    findings: List[str] = []
    if initial_spread_delta > 10.0:
        findings.append("unzeroed_spread_bias_over_10_deg")
    if mean_motion_delta > 0.25:
        findings.append("unzeroed_motion_confidence_higher")
    if mean_stop_delta < -0.25:
        findings.append("unzeroed_stop_probability_lower")

    return {
        "zeroed_sample_count": len(zeroed_rows),
        "unzeroed_sample_count": len(unzeroed_rows),
        "initial_spread_delta_deg": initial_spread_delta,
        "mean_spread_zeroed_deg": None if not zeroed_spread else mean(zeroed_spread),
        "mean_spread_unzeroed_deg": None if not unzeroed_spread else mean(unzeroed_spread),
        "mean_motion_confidence_delta": mean_motion_delta,
        "mean_stop_probability_delta": mean_stop_delta,
        "status": "WARN" if findings else "PASS",
        "findings": findings,
    }


def build_report(
    csv_path: Path,
    *,
    side: str,
    zeroed_csv: Optional[Path] = None,
    unzeroed_csv: Optional[Path] = None,
) -> Dict[str, object]:
    rows = load_rows(csv_path)
    report: Dict[str, object] = {
        "log_path": str(csv_path),
        "fixed_link": summarize_fixed_link(rows, side=side),
        "stop_response": summarize_stop_response(rows),
    }
    if zeroed_csv and unzeroed_csv:
        report["zeroing_comparison"] = compare_zeroing(load_rows(zeroed_csv), load_rows(unzeroed_csv))
    return report


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path, help="Runtime CSV from the fixed-link or stop-leg trial")
    parser.add_argument("--side", choices=sorted(SIDE_PREFIX), default="left", help="Fixed side to summarize")
    parser.add_argument("--zeroed-csv", type=Path, default=None, help="Correctly zeroed CSV for A/B comparison")
    parser.add_argument("--unzeroed-csv", type=Path, default=None, help="Unzeroed CSV for A/B comparison")
    parser.add_argument("--output", type=Path, default=None, help="Write JSON report to this path")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    report = build_report(
        args.csv,
        side=args.side,
        zeroed_csv=args.zeroed_csv,
        unzeroed_csv=args.unzeroed_csv,
    )
    text = json.dumps(report, indent=2, ensure_ascii=False)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text + "\n", encoding="utf-8")
    else:
        print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
