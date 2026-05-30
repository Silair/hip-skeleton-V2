#!/usr/bin/env python3
"""Generate hs_exoskeleton_v2 replay CSV from a single-leg MuJoCo model.

The tool drives one MuJoCo hinge joint with an optional sinusoidal PD "human"
actuator and records joint position/velocity into the existing offline replay
CSV contract. It never applies V2 controller torque to the MuJoCo model.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Dict, Iterable, Optional


FIELDNAMES = [
    "TimeS",
    "LeftJointPosRad",
    "RightJointPosRad",
    "LeftJointVelRadS",
    "RightJointVelRadS",
    "Healthy",
    "Enabled",
]


def clip(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def format_float(value: float) -> str:
    return f"{value:.12g}"


def compute_pd_torque(
    *,
    qpos: float,
    qvel: float,
    time_s: float,
    amplitude_rad: float,
    frequency_hz: float,
    kp: float,
    kd: float,
    torque_limit: float,
) -> float:
    omega = 2.0 * math.pi * frequency_hz
    q_des = amplitude_rad * math.sin(omega * time_s)
    qvel_des = amplitude_rad * omega * math.cos(omega * time_s)
    torque = kp * (q_des - qpos) + kd * (qvel_des - qvel)
    return clip(torque, -abs(torque_limit), abs(torque_limit))


def make_replay_row(
    *,
    time_s: float,
    qpos: float,
    qvel: float,
    virtual_right: str,
) -> Dict[str, str]:
    if virtual_right == "zero":
        right_pos = 0.0
        right_vel = 0.0
    elif virtual_right == "copy":
        right_pos = qpos
        right_vel = qvel
    elif virtual_right == "inverted":
        right_pos = -qpos
        right_vel = -qvel
    else:
        raise ValueError(f"unsupported virtual_right mode: {virtual_right}")

    return {
        "TimeS": f"{time_s:.9f}",
        "LeftJointPosRad": format_float(qpos),
        "RightJointPosRad": format_float(right_pos),
        "LeftJointVelRadS": format_float(qvel),
        "RightJointVelRadS": format_float(right_vel),
        "Healthy": "1",
        "Enabled": "1",
    }


def required_id(mujoco, model, obj_type, name: str) -> int:
    obj_id = mujoco.mj_name2id(model, obj_type, name)
    if obj_id < 0:
        raise ValueError(f"missing {obj_type.name} named {name!r}")
    return int(obj_id)


def run_mujoco_curve(
    *,
    mjcf: Path,
    output: Path,
    joint: str,
    duration_s: float,
    sample_rate_hz: float,
    virtual_right: str,
    driver_actuator: Optional[str],
    amplitude_deg: float,
    frequency_hz: float,
    kp: float,
    kd: float,
    torque_limit: float,
) -> int:
    try:
        import mujoco
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "Python package 'mujoco' is not available. Run with the MuJoCo "
            "environment, e.g. /home/lin/miniconda3/envs/mujoco/bin/python."
        ) from exc

    model = mujoco.MjModel.from_xml_path(str(mjcf.expanduser()))
    data = mujoco.MjData(model)

    joint_id = required_id(mujoco, model, mujoco.mjtObj.mjOBJ_JOINT, joint)
    qpos_addr = int(model.jnt_qposadr[joint_id])
    qvel_addr = int(model.jnt_dofadr[joint_id])
    actuator_id = None
    if driver_actuator:
        actuator_id = required_id(mujoco, model, mujoco.mjtObj.mjOBJ_ACTUATOR, driver_actuator)

    output = output.expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)

    sample_dt = 1.0 / sample_rate_hz
    next_sample_s = 0.0
    amplitude_rad = math.radians(amplitude_deg)
    rows = 0

    with output.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDNAMES)
        writer.writeheader()

        while data.time <= duration_s + 1e-12:
            if actuator_id is not None:
                data.ctrl[actuator_id] = compute_pd_torque(
                    qpos=float(data.qpos[qpos_addr]),
                    qvel=float(data.qvel[qvel_addr]),
                    time_s=float(data.time),
                    amplitude_rad=amplitude_rad,
                    frequency_hz=frequency_hz,
                    kp=kp,
                    kd=kd,
                    torque_limit=torque_limit,
                )

            if data.time + 1e-12 >= next_sample_s:
                writer.writerow(
                    make_replay_row(
                        time_s=float(data.time),
                        qpos=float(data.qpos[qpos_addr]),
                        qvel=float(data.qvel[qvel_addr]),
                        virtual_right=virtual_right,
                    )
                )
                rows += 1
                next_sample_s += sample_dt

            mujoco.mj_step(model, data)

    return rows


def parse_args(argv: Optional[Iterable[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Export a single-leg MuJoCo dry-run replay CSV.")
    parser.add_argument("--mjcf", type=Path, required=True, help="Path to the MuJoCo MJCF XML model.")
    parser.add_argument("--output", type=Path, required=True, help="Output replay CSV path.")
    parser.add_argument("--joint", default="hip", help="Hinge joint to sample.")
    parser.add_argument("--duration-s", type=float, default=12.0)
    parser.add_argument("--sample-rate-hz", type=float, default=50.0)
    parser.add_argument("--virtual-right", choices=["zero", "copy", "inverted"], default="zero")
    parser.add_argument("--driver-actuator", default="", help="Optional actuator used to drive the sampled joint.")
    parser.add_argument("--amplitude-deg", type=float, default=25.0)
    parser.add_argument("--frequency-hz", type=float, default=0.8)
    parser.add_argument("--driver-kp", type=float, default=35.0)
    parser.add_argument("--driver-kd", type=float, default=4.0)
    parser.add_argument("--driver-torque-limit", type=float, default=80.0)
    args = parser.parse_args(argv)

    if args.duration_s <= 0.0:
        parser.error("--duration-s must be positive")
    if args.sample_rate_hz <= 0.0:
        parser.error("--sample-rate-hz must be positive")
    if args.frequency_hz <= 0.0:
        parser.error("--frequency-hz must be positive")
    if args.driver_torque_limit <= 0.0:
        parser.error("--driver-torque-limit must be positive")
    return args


def main(argv: Optional[Iterable[str]] = None) -> int:
    args = parse_args(argv)
    rows = run_mujoco_curve(
        mjcf=args.mjcf,
        output=args.output,
        joint=args.joint,
        duration_s=args.duration_s,
        sample_rate_hz=args.sample_rate_hz,
        virtual_right=args.virtual_right,
        driver_actuator=args.driver_actuator or None,
        amplitude_deg=args.amplitude_deg,
        frequency_hz=args.frequency_hz,
        kp=args.driver_kp,
        kd=args.driver_kd,
        torque_limit=args.driver_torque_limit,
    )
    print(f"wrote MuJoCo dry-run replay curve: {args.output.expanduser()} ({rows} samples)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
