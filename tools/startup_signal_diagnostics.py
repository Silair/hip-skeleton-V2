#!/usr/bin/env python3
"""Diagnose why anchor candidates are missing during multi_rate startup (e.g. 0.6 Hz).

Answers: spread/velocity too low vs thresholds, or not enough time for a half-period.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from statistics import mean
from typing import Dict, List, Optional, Sequence

MULTI_RATE_STARTUP_SEGMENT = (0.0, 4.0, 0.6)


def fnum(row: Dict[str, str], key: str, default: float = 0.0) -> float:
    try:
        return float(row.get(key, default))
    except (TypeError, ValueError):
        return default


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


def load_rows(path: Path) -> List[Dict[str, str]]:
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def filter_time_window(rows: List[Dict[str, str]], start_s: float, end_s: float) -> List[Dict[str, str]]:
    return [row for row in rows if start_s <= fnum(row, "MonoTimeS") < end_s]


def count_velocity_sign_changes(velocities: Sequence[float]) -> int:
    changes = 0
    prev_sign = 0
    for velocity in velocities:
        if abs(velocity) < 1e-6:
            continue
        sign = 1 if velocity > 0 else -1
        if prev_sign != 0 and sign != prev_sign:
            changes += 1
        prev_sign = sign
    return changes


def count_zero_crossings(velocities: Sequence[float]) -> int:
    crossings = 0
    prev = velocities[0] if velocities else 0.0
    for velocity in velocities[1:]:
        if prev > 0.0 and velocity <= 0.0:
            crossings += 1
        elif prev < 0.0 and velocity >= 0.0:
            crossings += 1
        prev = velocity
    return crossings


def peak_to_peak(values: Sequence[float]) -> float:
    if not values:
        return 0.0
    return max(values) - min(values)


def diagnose_segment(
    rows: List[Dict[str, str]],
    *,
    anchor_min_spread_deg: float,
    anchor_min_velocity_deg_s: float,
    segment_label: str,
    target_frequency_hz: Optional[float],
) -> Dict[str, object]:
    spreads = [fnum(row, "SpreadDeg") for row in rows]
    velocities_abs = [abs(fnum(row, "SignedPhaseVelocityDegS")) for row in rows]
    phase_signals = [fnum(row, "PhaseSignalRad") for row in rows]
    filtered_signals = [fnum(row, "FilteredPhaseSignalRad") for row in rows]
    signed_velocities = [fnum(row, "SignedPhaseVelocityDegS") for row in rows]

    candidate_count = sum(1 for row in rows if fnum(row, "AnchorCandidate", 0.0) >= 0.5)
    detected_count = sum(1 for row in rows if fnum(row, "AnchorDetected", 0.0) >= 0.5)
    startup_prior_candidate_count = sum(1 for row in rows if fnum(row, "StartupPriorCandidate", 0.0) >= 0.5)
    startup_prior_applied_count = sum(1 for row in rows if fnum(row, "StartupPriorApplied", 0.0) >= 0.5)

    spread_p95 = percentile(spreads, 95.0)
    velocity_p95 = percentile(velocities_abs, 95.0)
    spread_max = max(spreads) if spreads else 0.0
    velocity_max = max(velocities_abs) if velocities_abs else 0.0

    duration_s = 0.0
    if rows:
        duration_s = max(0.0, fnum(rows[-1], "MonoTimeS") - fnum(rows[0], "MonoTimeS"))

    half_period_s = None
    if target_frequency_hz and target_frequency_hz > 1e-6:
        half_period_s = 0.5 / target_frequency_hz

    spread_ok = spread_p95 is not None and spread_p95 >= anchor_min_spread_deg
    velocity_ok = velocity_p95 is not None and velocity_p95 >= anchor_min_velocity_deg_s
    zero_cross = count_zero_crossings(signed_velocities)

    likely_causes: List[str] = []
    if spread_p95 is not None and spread_p95 < anchor_min_spread_deg:
        likely_causes.append("amplitude_low_spread_below_threshold")
    if velocity_p95 is not None and velocity_p95 < anchor_min_velocity_deg_s:
        likely_causes.append("velocity_low_below_threshold")
    if half_period_s is not None and duration_s < half_period_s * 0.9:
        likely_causes.append("insufficient_time_for_half_period")
    if candidate_count == 0 and spread_ok and velocity_ok and zero_cross > 0:
        likely_causes.append("signal_ok_but_anchor_candidate_gate_failed")
    elif candidate_count == 0 and not likely_causes:
        likely_causes.append("no_candidate_other_gate_or_confirm")

    return {
        "segment_label": segment_label,
        "sample_count": len(rows),
        "duration_s": duration_s,
        "target_frequency_hz": target_frequency_hz,
        "expected_half_period_s": half_period_s,
        "spread_deg_max": spread_max,
        "spread_deg_p95": spread_p95,
        "spread_deg_mean": None if not spreads else mean(spreads),
        "velocity_abs_max": velocity_max,
        "velocity_abs_p95": velocity_p95,
        "velocity_abs_mean": None if not velocities_abs else mean(velocities_abs),
        "phase_signal_amplitude_rad": peak_to_peak(phase_signals),
        "filtered_phase_signal_amplitude_rad": peak_to_peak(filtered_signals),
        "zero_cross_count": zero_cross,
        "velocity_sign_change_count": count_velocity_sign_changes(signed_velocities),
        "candidate_count": candidate_count,
        "anchor_detected_count": detected_count,
        "startup_prior_candidate_count": startup_prior_candidate_count,
        "startup_prior_applied_count": startup_prior_applied_count,
        "anchor_min_spread_deg": anchor_min_spread_deg,
        "anchor_min_velocity_deg_s": anchor_min_velocity_deg_s,
        "spread_below_threshold": spread_p95 is not None and spread_p95 < anchor_min_spread_deg,
        "velocity_below_threshold": velocity_p95 is not None and velocity_p95 < anchor_min_velocity_deg_s,
        "likely_causes": likely_causes,
    }


def startup_signal_diagnostics(
    rows: List[Dict[str, str]],
    *,
    start_s: float = MULTI_RATE_STARTUP_SEGMENT[0],
    end_s: float = MULTI_RATE_STARTUP_SEGMENT[1],
    target_frequency_hz: float = MULTI_RATE_STARTUP_SEGMENT[2],
    anchor_min_spread_deg: float = 4.0,
    anchor_min_velocity_deg_s: float = 8.0,
) -> Dict[str, object]:
    segment_rows = filter_time_window(rows, start_s, end_s)
    label = f"{target_frequency_hz:.2f}Hz_{start_s:.0f}-{end_s:.0f}s"
    return diagnose_segment(
        segment_rows,
        anchor_min_spread_deg=anchor_min_spread_deg,
        anchor_min_velocity_deg_s=anchor_min_velocity_deg_s,
        segment_label=label,
        target_frequency_hz=target_frequency_hz,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument("--start-s", type=float, default=MULTI_RATE_STARTUP_SEGMENT[0])
    parser.add_argument("--end-s", type=float, default=MULTI_RATE_STARTUP_SEGMENT[1])
    parser.add_argument("--target-frequency-hz", type=float, default=MULTI_RATE_STARTUP_SEGMENT[2])
    parser.add_argument("--anchor-min-spread-deg", type=float, default=4.0)
    parser.add_argument("--anchor-min-velocity-deg-s", type=float, default=8.0)
    parser.add_argument("--json-output", type=Path)
    args = parser.parse_args()

    summary = startup_signal_diagnostics(
        load_rows(args.csv),
        start_s=args.start_s,
        end_s=args.end_s,
        target_frequency_hz=args.target_frequency_hz,
        anchor_min_spread_deg=args.anchor_min_spread_deg,
        anchor_min_velocity_deg_s=args.anchor_min_velocity_deg_s,
    )

    print(f"segment: {summary['segment_label']}")
    print(f"  spread_deg: max={summary['spread_deg_max']:.2f} p95={summary['spread_deg_p95']:.2f} (min {summary['anchor_min_spread_deg']})")
    print(
        f"  velocity_abs: max={summary['velocity_abs_max']:.2f} p95={summary['velocity_abs_p95']:.2f} "
        f"(min {summary['anchor_min_velocity_deg_s']})"
    )
    print(f"  filtered_amp_rad: {summary['filtered_phase_signal_amplitude_rad']:.4f}")
    print(f"  zero_cross={summary['zero_cross_count']} sign_changes={summary['velocity_sign_change_count']}")
    print(
        f"  candidates={summary['candidate_count']} detected={summary['anchor_detected_count']} "
        f"startup_prior_cand={summary.get('startup_prior_candidate_count', 0)} "
        f"startup_prior_applied={summary.get('startup_prior_applied_count', 0)}"
    )
    print(f"  likely_causes: {summary['likely_causes']}")

    if args.json_output:
        args.json_output.parent.mkdir(parents=True, exist_ok=True)
        args.json_output.write_text(json.dumps(summary, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
