# Control Effect Evaluation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** Add an offline PASS/FAIL control-effect evaluator on top of existing hs_exoskeleton_v2 runtime metrics.

**Architecture:** Keep evaluation outside the realtime controller. `tools/analyze_run.py` computes metrics as before, then evaluates them against configurable thresholds, writes `evaluation.json`, renders an HTML judgement table, and optionally exits non-zero for automation.

**Tech Stack:** Python standard library, existing CSV/HTML report flow, existing `unittest` tests.

---

### Task 1: Add evaluation model and default thresholds

**Files:**
- Modify: `tools/analyze_run.py`
- Test: `tests/test_analyze_run.py`

- [x] Add `DEFAULT_EVALUATION_THRESHOLDS` with conservative defaults matching `RUNTIME_ANALYSIS.md`.
- [x] Add `compare_metric()` and `evaluate_metrics()` helpers returning structured checks.
- [x] Treat missing required reference metrics as failed/insufficient data.

### Task 2: Wire evaluation into reports and CLI

**Files:**
- Modify: `tools/analyze_run.py`

- [x] Extend `write_reports()` to accept evaluation, write `evaluation.json`, and render an HTML section.
- [x] Extend `analyze_log()` to build thresholds from CLI/defaults and return evaluation.
- [x] Add CLI options for threshold overrides and `--fail-on-evaluation`.
- [x] Preserve existing metrics/report behavior.

### Task 3: Add regression tests

**Files:**
- Modify: `tests/test_analyze_run.py`

- [x] Assert synthetic stable gait produces PASS.
- [x] Assert report directory contains `evaluation.json` and HTML contains the judgement section.
- [x] Add direct evaluator failure test for torque limit and freeze violation.

### Task 4: Verify

**Commands:**
- `python3 -m unittest tests/test_analyze_run.py`
- `python3 tools/analyze_run.py <synthetic csv> --output-dir <tmp/report> --fail-on-evaluation` through the unit path where feasible.


### Task 5: Document decisions

**Files:**
- Modify: `RUNTIME_ANALYSIS.md`
- Modify: `README.md`

- [x] Document generated `evaluation.json`, HTML judgement section, CLI `evaluation_status`, threshold overrides, and `--fail-on-evaluation`.
- [x] Include the offline-only disclaimer and approximate-reference-phase limitation.
