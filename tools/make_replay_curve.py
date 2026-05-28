#!/usr/bin/env python3
"""Generate offline replay input curves for hs_exoskeleton_v2.

The output CSV is consumed by tools/replay_control.cpp / the
hs_exoskeleton_v2_replay executable.  No third-party dependencies are required.
"""

from __future__ import annotations

import argparse
import csv
import math
import random
from pathlib import Path
from typing import Callable, Dict, Iterable, List, Tuple

TAU = 2.0 * math.pi


def _safe_env(t: float) -> Dict[str, object]:
    allowed = {
        "abs": abs,
        "min": min,
        "max": max,
        "pow": pow,
        "sin": math.sin,
        "cos": math.cos,
        "tan": math.tan,
        "asin": math.asin,
        "acos": math.acos,
        "atan": math.atan,
        "atan2": math.atan2,
        "sqrt": math.sqrt,
        "exp": math.exp,
        "log": math.log,
        "floor": math.floor,
        "ceil": math.ceil,
        "pi": math.pi,
        "tau": TAU,
        "t": t,
    }
    return allowed


def eval_expr(expr: str, t: float) -> float:
    return float(eval(expr, {"__builtins__": {}}, _safe_env(t)))


def active_envelope(t: float, duration: float) -> float:
    """A deterministic walk-stop-walk envelope for stop_go scenario."""
    first_stop = duration * 0.40
    resume = duration * 0.65
    if first_stop <= t < resume:
        return 0.0
    ramp = min(1.0, t / max(0.5, duration * 0.08))
    if t >= resume:
        ramp = min(1.0, (t - resume) / max(0.5, duration * 0.08))
    return ramp


def signal_for_scenario(args: argparse.Namespace, t: float) -> float:
    if args.scenario == "sine":
        phase = TAU * args.frequency_hz * t
        return args.amplitude_rad * math.sin(phase)
    if args.scenario == "stop_go":
        phase = TAU * args.frequency_hz * t
        return active_envelope(t, args.duration_s) * args.amplitude_rad * math.sin(phase)
    if args.scenario == "freq_ramp":
        # Linear frequency ramp from frequency_hz to ramp_end_frequency_hz.
        k = (args.ramp_end_frequency_hz - args.frequency_hz) / max(args.duration_s, 1e-9)
        phase = TAU * (args.frequency_hz * t + 0.5 * k * t * t)
        return args.amplitude_rad * math.sin(phase)
    raise ValueError(f"scenario {args.scenario!r} should be handled by expressions")


def generate_positions(args: argparse.Namespace) -> List[Tuple[float, float, float, int, int]]:
    rng = random.Random(args.seed)
    sample_count = int(round(args.duration_s * args.rate_hz)) + 1
    rows: List[Tuple[float, float, float, int, int]] = []

    for i in range(sample_count):
        t = i / args.rate_hz
        if args.left_expr or args.right_expr:
            if not (args.left_expr and args.right_expr):
                raise ValueError("--left-expr and --right-expr must be provided together")
            left = eval_expr(args.left_expr, t)
            right = eval_expr(args.right_expr, t)
        else:
            signal = signal_for_scenario(args, t)
            # GaitFeatureExtractor currently uses left + right as the phase signal.
            # Split the designed scalar signal evenly across both sides so the replay
            # produces a clear controllable phase signal under that convention.
            left = 0.5 * signal
            right = 0.5 * signal

        if args.noise_rad > 0.0:
            left += rng.gauss(0.0, args.noise_rad)
            right += rng.gauss(0.0, args.noise_rad)

        healthy = 1
        enabled = 1
        rows.append((t, left, right, healthy, enabled))
    return rows


def finite_difference(values: List[float], times: List[float], i: int) -> float:
    if len(values) < 2:
        return 0.0
    if i == 0:
        dt = times[1] - times[0]
        return 0.0 if dt <= 0.0 else (values[1] - values[0]) / dt
    if i == len(values) - 1:
        dt = times[-1] - times[-2]
        return 0.0 if dt <= 0.0 else (values[-1] - values[-2]) / dt
    dt = times[i + 1] - times[i - 1]
    return 0.0 if dt <= 0.0 else (values[i + 1] - values[i - 1]) / dt


def write_curve(path: Path, rows: List[Tuple[float, float, float, int, int]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    times = [row[0] for row in rows]
    left_positions = [row[1] for row in rows]
    right_positions = [row[2] for row in rows]

    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow([
            "TimeS",
            "LeftJointPosRad",
            "RightJointPosRad",
            "LeftJointVelRadS",
            "RightJointVelRadS",
            "Healthy",
            "Enabled",
        ])
        for i, (t, left, right, healthy, enabled) in enumerate(rows):
            writer.writerow([
                f"{t:.9f}",
                f"{left:.12g}",
                f"{right:.12g}",
                f"{finite_difference(left_positions, times, i):.12g}",
                f"{finite_difference(right_positions, times, i):.12g}",
                healthy,
                enabled,
            ])


def parse_args(argv: Iterable[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate a joint-angle curve CSV for V2 offline replay.")
    parser.add_argument("--output", required=True, help="Output curve CSV path.")
    parser.add_argument("--scenario", choices=["sine", "stop_go", "freq_ramp", "custom"], default="sine")
    parser.add_argument("--duration-s", type=float, default=12.0)
    parser.add_argument("--rate-hz", type=float, default=50.0)
    parser.add_argument("--amplitude-rad", type=float, default=0.35)
    parser.add_argument("--frequency-hz", type=float, default=0.8)
    parser.add_argument("--ramp-end-frequency-hz", type=float, default=1.2)
    parser.add_argument("--noise-rad", type=float, default=0.0)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--left-expr", help="Custom Python math expression using t, pi, tau, sin/cos/etc.")
    parser.add_argument("--right-expr", help="Custom Python math expression using t, pi, tau, sin/cos/etc.")
    args = parser.parse_args(argv)

    if args.duration_s <= 0.0:
        parser.error("--duration-s must be positive")
    if args.rate_hz <= 0.0:
        parser.error("--rate-hz must be positive")
    if args.scenario == "custom" and not (args.left_expr and args.right_expr):
        parser.error("custom scenario requires --left-expr and --right-expr")
    return args


def main(argv: Iterable[str] | None = None) -> int:
    args = parse_args(argv)
    rows = generate_positions(args)
    write_curve(Path(args.output), rows)
    print(f"wrote replay curve: {args.output} ({len(rows)} samples)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
