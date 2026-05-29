#!/usr/bin/env python3
"""Summarize anchor frequency updates over time from a V2 runtime CSV."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Dict, List

ASSIST_STATE_NAMES = {
    0: "Transparent",
    1: "Tracking",
    2: "Ramp",
    3: "Active",
    4: "Stopping",
    5: "Frozen",
    6: "Fault",
}

MULTI_RATE_SEGMENTS = [
    (0.0, 4.0, 0.6),
    (4.0, 8.0, 1.2),
    (8.0, 12.0, 0.75),
    (12.0, 16.0, 1.35),
    (16.0, 20.0, 0.9),
]


def fnum(row: Dict[str, str], key: str, default: float = 0.0) -> float:
    try:
        return float(row.get(key, default))
    except (TypeError, ValueError):
        return default


def load_rows(path: Path) -> List[Dict[str, str]]:
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def segment_label(t_s: float) -> str:
    for start, end, hz in MULTI_RATE_SEGMENTS:
        if start <= t_s < end:
            return f"{hz:.2f}Hz"
    return "out_of_range"


def summarize(path: Path) -> Dict[str, object]:
    rows = load_rows(path)
    events: List[Dict[str, object]] = []
    segment_updates: Dict[str, int] = {f"{hz:.2f}Hz": 0 for _, _, hz in MULTI_RATE_SEGMENTS}

    for row in rows:
        if fnum(row, "AnchorFrequencyUpdated", 0.0) < 0.5:
            continue
        t_s = fnum(row, "MonoTimeS")
        assist = int(fnum(row, "AssistState", -1))
        seg = segment_label(t_s)
        if seg in segment_updates:
            segment_updates[seg] += 1
        events.append(
            {
                "time_s": t_s,
                "assist_state": ASSIST_STATE_NAMES.get(assist, str(assist)),
                "stop_probability": fnum(row, "StopProbability"),
                "measured_frequency_hz": fnum(row, "AnchorMeasuredFrequencyHz"),
                "omega_correction_hz": fnum(row, "OmegaCorrectionHz"),
                "frequency_hz": fnum(row, "Frequency"),
                "anchor_confidence": fnum(row, "AnchorConfidence"),
                "segment": seg,
            }
        )

    edge_updates = [
        e
        for e in events
        if e["assist_state"] == "Stopping"
        or e["stop_probability"] >= 0.8
        or (e["stop_probability"] >= 0.65 and e["assist_state"] in ("Active", "Ramp"))
    ]

    return {
        "log_path": str(path),
        "anchor_update_count": len(events),
        "updates_by_segment": segment_updates,
        "events": events,
        "edge_or_stop_context_updates": edge_updates,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument("--json-output", type=Path)
    args = parser.parse_args()

    summary = summarize(args.csv)
    print(f"log: {summary['log_path']}")
    print(f"anchor updates: {summary['anchor_update_count']}")
    print("by segment:", summary["updates_by_segment"])
    if summary["edge_or_stop_context_updates"]:
        print("edge/stop-context updates:")
        for event in summary["edge_or_stop_context_updates"]:
            print(
                f"  t={event['time_s']:.2f}s state={event['assist_state']} "
                f"stop_p={event['stop_probability']:.2f} meas={event['measured_frequency_hz']:.3f}Hz "
                f"d_omega={event['omega_correction_hz']:+.3f}Hz conf={event['anchor_confidence']:.2f}"
            )
    else:
        print("edge/stop-context updates: none")

    if args.json_output:
        args.json_output.parent.mkdir(parents=True, exist_ok=True)
        args.json_output.write_text(json.dumps(summary, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
