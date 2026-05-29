# V2.2 Anchor 诊断、延迟确认与 Tracking 延迟频率 — 报告

**日期：** 2026-05-29  
**对比对象：** V2.1（`43051d2` / `artifacts/v21_anchor_ab/after`） vs V2.2  
**方法：** 相同 8 条标准回放曲线 → `benchmark_v21_anchor_batch.sh`

## 定位（合入说明）

```text
V2.2 does not significantly improve multi_rate metrics over V2.1, but adds anchor reject
diagnostics and confirms that 0.6Hz startup has no reliable anchor candidates due to UnreliableSignal.
```

中文：

```text
V2.2 相比 V2.1 性能提升很小；价值在于诊断闭环——确认 0.6Hz 起步段没有 anchor 的根因是
信号不可靠（UnreliableSignal），而不是状态机或 confidence 参数问题。
```

V2.2 是 **诊断增强 + 1 帧延迟确认 + Tracking deferred frequency 框架**，不是性能大提升版。

---

## 1. 实现摘要

| 能力 | 说明 |
|------|------|
| `AnchorRejectReason` | 拒绝原因枚举 + CSV `AnchorRejectReason` + `analyze_run` 聚合 |
| 延迟 1 帧确认 | `anchor_confirm_delay_frames = 1`，失败记 `ConfirmFailed` |
| Tracking 延迟频率 | `enable_tracking_deferred_frequency`：Tracking 测得可靠半周期后，Ramp/Active 首帧应用 |
| 分段诊断 | `analyze_anchor_timeline.py` → `segment_diagnostics`（候选/检测/更新/拒绝） |
| `AnchorCandidate` | CSV 列，标记延迟确认前的可靠过零候选 |

**未做（仍属 V2.2+）：** `anchor_phase_gain` 相位校正、stride memory。

---

## 2. `multi_rate`：V2.1 → V2.2

| 指标 | V2.1 | V2.2 | 变化 |
|------|------|------|------|
| `frequency_rmse_hz` | 0.2063 | 0.2058 | ≈ −0.2% |
| `frequency_adaptation_time_mean_s` | 1.27 | 1.24 | −2.6% |
| `phase_rmse_percent` | 11.57 | 11.56 | ≈ 持平 |
| `peak_torque_phase_mae_deg` | 3.56 | 3.46 | −2.8% |
| `anchor_update_count` | 15 | 15 | 持平 |
| `anchor_update_during_stop_count` | 0 | 0 | 安全硬门槛 |
| `omega_jump_max_hz` | 0.025 | 0.025 | 持平 |

详细对比见 [`multi_rate_v21_to_v22.md`](multi_rate_v21_to_v22.md)。

---

## 3. 0.6 Hz 段诊断（TODO-1 结论）

`multi_rate` 第一段（0–4 s，0.6 Hz）**仍为 `anchor_update_count = 0`**。

分段统计（`multi_rate_timeline.json`）：

| 段 | 候选 | 检测 | 更新 | 主要拒绝原因 |
|----|------|------|------|----------------|
| 0.60 Hz | 0 | 0 | 0 | **UnreliableSignal**（5） |
| 1.20 Hz | 9 | 9 | 3 | Warmup、LowConfidence |
| 0.75 Hz | 1 | 1 | 1 | UnreliableSignal |
| 1.35 Hz | 11 | 11 | 8 | Interval、LowConfidence |
| 0.90 Hz | 4 | 4 | 3 | UnreliableSignal |

**结论（分段 + 起步信号诊断）：**

1. 分段：0.6 Hz 段 `candidate=0`，拒绝以 `UnreliableSignal` 为主（过零但未同时满足可靠峰/谷门控）。
2. `startup_signal_diagnostics`：该段 `spread_deg_p95≈19.9`、`velocity_abs_p95≈74.9`，**均高于**默认门控（4° / 8°/s）→ **不是** 幅度/速度整体不足（排除情况 A/B 作为主因）。
3. 更可能：**延迟确认 / refractory / tracking 门控 / 可靠过零瞬时条件** 导致无 `AnchorCandidate`（情况 C 或门控组合）。

**建议（V2.2.1，勿盲目降 `anchor_min_confidence`）：** startup frequency prior 或 partial stride；见路线图，不要混进稳态 anchor。

---

## 4. 停步上下文指标（V2.2 收口后）

已拆分为三项（`analyze_run.py`）：

| 指标 | 含义 | 安全硬门槛 |
|------|------|------------|
| `anchor_update_during_stop_count` | 停步段实际更新频率 | **必须为 0** |
| `anchor_rejected_during_stop_count` | 停步段看见 anchor 但被拒绝 | 信息性 |
| `anchor_candidate_during_stop_count` | 停步段过零候选 | 信息性 |

`candidate/rejected > 0` 且 `update = 0` 表示抖动被门控挡住，**不等于控制危险**。

停步上下文定义：`AssistState == Stopping` 或 `StopProbability >= 0.8`。

## 5. 0.6 Hz 起步信号诊断

```bash
python3 tools/startup_signal_diagnostics.py /path/to/multi_rate_runtime.csv \
  --json-output artifacts/v22_anchor/multi_rate_startup_signal.json
```

输出：`spread_deg_*`、`velocity_abs_*`、过零计数、`likely_causes`（幅度不足 / 速度不足 / 半周期时间不足）。

---

## 6. 复现

```bash
g++ -std=c++17 -I. -I/path/to/MultiHarmonicAO/include tools/replay_control.cpp \
  control/*.cpp logging/ExoLogger.cpp -o /tmp/hsx_v22_replay

./tools/benchmark_v21_anchor_batch.sh /tmp/hsx_v22_replay /tmp/v22_bench
python3 tools/analyze_anchor_timeline.py /tmp/v22_bench/logs/.../multi_rate_*.csv
python3 tools/compare_metrics.py artifacts/v21_anchor_ab/after/multi_rate.json \
  artifacts/v22_anchor/multi_rate.json
```

---

## 7. 合入建议

**可合入 V2.2 第一版**（在 V2.1 之上）：诊断与延迟确认不破坏 `multi_rate` 核心指标；Tracking 延迟频率为 Ramp 过渡预留，**未解决** 0.6 Hz 无候选问题。

合入说明：

```text
V2.2 增加 anchor 拒绝诊断与 1 帧抗噪确认；Tracking→Ramp 可应用缓存频率；
0.6 Hz 起步仍依赖信号门控达标，需后续针对 UnreliableSignal 单独设计。
```
