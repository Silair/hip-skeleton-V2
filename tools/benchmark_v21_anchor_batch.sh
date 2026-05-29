#!/usr/bin/env bash
# Run standard offline replay curves and write metrics.json per scenario.
# Usage:
#   ./tools/benchmark_v21_anchor_batch.sh /tmp/replay_bin /tmp/output_dir

set -euo pipefail

REPLAY_BIN="${1:?replay binary path}"
OUT_DIR="${2:?output directory}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CURVES_DIR="${OUT_DIR}/curves"
mkdir -p "${CURVES_DIR}" "${OUT_DIR}/logs" "${OUT_DIR}/metrics"

AO_INCLUDE="${AO_INCLUDE:-/home/lin/code/2026.4.14hs_exoskeleton/include}"

run_curve() {
  local name="$1"
  shift
  local csv="${CURVES_DIR}/${name}.csv"
  python3 "${ROOT}/tools/make_replay_curve.py" --output "${csv}" "$@"
  "${REPLAY_BIN}" --input "${csv}" --output-dir "${OUT_DIR}/logs" --prefix "${name}"
  local log
  log="$(find "${OUT_DIR}/logs" -name "${name}_*.csv" | sort | tail -1)"
  python3 "${ROOT}/tools/analyze_run.py" "${log}" --output-dir "${OUT_DIR}/metrics/${name}"
  cp "${OUT_DIR}/metrics/${name}/metrics.json" "${OUT_DIR}/metrics/${name}.json"
  echo "OK ${name} -> ${log}"
}

run_curve steady_0p8 --scenario sine --duration-s 12 --rate-hz 50 --amplitude-rad 0.35 --frequency-hz 0.8
run_curve freq_ramp --scenario freq_ramp --duration-s 18 --rate-hz 50 --amplitude-rad 0.35 --frequency-hz 0.6 --ramp-end-frequency-hz 1.2
run_curve amp_ramp --scenario amplitude_ramp --duration-s 18 --rate-hz 50 --amplitude-rad 0.40 --min-amplitude-rad 0.08 --frequency-hz 0.8
run_curve stop_go --scenario stop_go --duration-s 18 --rate-hz 50 --amplitude-rad 0.35 --frequency-hz 0.8
run_curve abrupt_stop --scenario abrupt_stop --duration-s 12 --rate-hz 50 --amplitude-rad 0.35 --frequency-hz 0.8 --stop-time-s 5.2
run_curve asymmetric --scenario asymmetric --duration-s 12 --rate-hz 50 --amplitude-rad 0.35 --frequency-hz 0.8
run_curve repeated_stop_go --scenario repeated_stop_go --duration-s 24 --rate-hz 50 --amplitude-rad 0.35 --frequency-hz 0.8 --stop-cycle-s 4.0 --stop-window-s 1.0
run_curve multi_rate --scenario multi_rate --duration-s 20 --rate-hz 50 --amplitude-rad 0.35 --rate-sequence-hz 0.6,1.2,0.75,1.35,0.9

echo "Metrics written under ${OUT_DIR}/metrics/"
