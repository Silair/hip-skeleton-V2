# V2.2 / V2.2.1 改动总结

**仓库：** `hs_exoskeleton_v2`  
**日期：** 2026-05-29  
**本地提交：** V2.1 `43051d2` → V2.2 `584046e` → V2.2.1（本文件对应之提交）  
**未推送远端**

---

## 总览

| 版本 | 定位 | 性能预期 |
|------|------|----------|
| **V2.2** | Anchor 诊断 + 1 帧延迟确认 + Tracking 延迟频率框架 | 相对 V2.1 几乎不提升 `multi_rate` 指标；确认 0.6 Hz 无 candidate 根因 |
| **V2.2.1** | 起步频率先验 `StartupPrior`（独立于稳态 anchor） | `multi_rate` `frequency_rmse_hz` 略降；0–4 s 段可初始化 `omega_target` |

**明确不做：** `anchor_phase_gain`、完整双层 AO（待 V2.3 stride 日志版稳定后再议）。

---

## V2.2（commit `584046e`）

### 控制与日志

- **`AnchorRejectReason` 枚举** → `PhaseEstimate::anchor_reject_reason` + CSV `AnchorRejectReason`
- **`anchor_confirm_delay_frames = 1`**：可靠过零候选延迟 1 帧确认；失败记 `ConfirmFailed`
- **`enable_tracking_deferred_frequency`**：Tracking 段测得可靠半周期后缓存，Ramp/Active 首帧应用一次
- **CSV `AnchorCandidate`**：延迟确认前的候选标记

### 分析与报告

- `analyze_run.py`：`anchor_reject_reason_counts`
- **停步上下文指标拆分**（替代易误导的 `false_anchor_during_stop_count`）：
  - `anchor_update_during_stop_count` — **安全硬门槛 = 0**
  - `anchor_rejected_during_stop_count` — 信息性
  - `anchor_candidate_during_stop_count` — 信息性
- `analyze_anchor_timeline.py`：`segment_diagnostics`（`multi_rate` 各 4 s 段）
- `tools/startup_signal_diagnostics.py`：0.6 Hz 起步段 spread/velocity/过零统计
- `artifacts/v22_anchor/V22_ANCHOR_REPORT.md`：合入说明与离线对比

### 主要结论（`multi_rate`）

- V2.2 相比 V2.1 性能提升很小
- 0.6 Hz 段：`anchor_update = 0`，`candidate = 0`；**不是** AssistState/confidence 问题
- `startup_signal_diagnostics`：spread/velocity 达标，但 `signal_ok_but_anchor_candidate_gate_failed`（可靠过零 + 延迟确认等门控）

---

## V2.2.1（本批未提交改动）

### 设计

见 [`V22_1_STARTUP_PRIOR.md`](V22_1_STARTUP_PRIOR.md)。

**原则：** 起步先验 ≠ 正式 anchor；不生成 `AnchorDetected`、不改 phase offset；只混合 `omega_target`（仍受 `max_omega_rate_rad_s2` 限速）。

### 检测条件（Tracking / Ramp）

- `stop_probability < 0.55`（适配起步段 intent 滞后）
- `motion_confidence ≥ 0.48`（`motion_confidence_rise` 默认 0，允许平台期）
- 同向速度连续 **4** 帧（`startup_prior_min_velocity_deg_s = 5`）
- spread 递增连续 **3** 帧

### 频率估计与应用

- **仅用过零半周期**估计 `startup_frequency_prior_hz`（`[0.45, 1.0] Hz`）；已移除 velocity 粗估，避免过早拉到 0.45 Hz
- **应用一次**：`startup_prior_min_apply_time_s = 0.5` 后
  - Ramp/Active：`startup_prior_gain = 0.20` × confidence
  - Tracking（默认开）：× `startup_prior_tracking_gain_scale = 0.50`
- `PhaseEstimator::update` 新增参数 `motion_confidence`（上一帧 intent）

### 新增 CSV 列

`StartupPriorValid`, `StartupPriorCandidate`, `StartupPriorApplied`, `StartupPriorFrequencyHz`, `StartupPriorConfidence`

### 新增指标（`analyze_run.py`）

- `startup_prior_candidate_count`
- `startup_prior_applied_count`
- `startup_prior_frequency_hz_mean`

`startup_signal_diagnostics.py` 增加段内 `startup_prior_*` 计数。

### 修改文件一览

| 路径 | 说明 |
|------|------|
| `config/ControlConfig.h` | StartupPrior 参数 |
| `control/PhaseEstimator.{h,cpp}` | 检测、混合、应用逻辑 |
| `control/GaitFeatures.h` | `PhaseEstimate` 起步字段 |
| `control/ExoController.{h,cpp}` | `last_motion_confidence_` |
| `tools/replay_control.cpp` | 同上 |
| `logging/ExoLogger.cpp` | CSV 列 |
| `tools/analyze_run.py` | 指标 |
| `tools/startup_signal_diagnostics.py` | 段内 prior 统计 |
| `tests/test_control.cpp` | 过零先验、Ramp 应用、高 stop 跳过 |
| `CODE_WIKI.md`, `RUNTIME_ANALYSIS.md` | 文档索引 |

### 离线验证（`multi_rate`，V2.2 → V2.2.1）

| 指标 | V2.2 | V2.2.1 |
|------|------|--------|
| `frequency_rmse_hz` | 0.206 | **0.204** |
| `startup_prior_applied`（全段） | — | 1 |
| **0–4 s 段 applied** | 0 | **1**（t≈1.28 s，prior≈**0.61 Hz**） |
| `anchor_update_during_stop_count` | 0 | 0 |

### 单元测试

- `test_phase_estimator_builds_startup_prior_from_zero_cross`
- `test_phase_estimator_applies_startup_prior_on_ramp_entry`
- `test_phase_estimator_startup_prior_skipped_when_stop_probability_high`

---

## 复现命令

```bash
# 编译 replay
g++ -std=c++17 -I. -I/path/to/MultiHarmonicAO/include \
  tools/replay_control.cpp control/AssistStateMachine.cpp control/FreezeManager.cpp \
  control/GaitFeatureExtractor.cpp control/IntentDetector.cpp control/PhaseEstimator.cpp \
  control/TorqueProfile.cpp control/StopDetector.cpp control/StopTorqueLimiter.cpp \
  logging/ExoLogger.cpp -o /tmp/hsx_replay

./tools/benchmark_v21_anchor_batch.sh /tmp/hsx_replay /tmp/bench_out

python3 tools/startup_signal_diagnostics.py /tmp/bench_out/logs/.../multi_rate_*.csv
python3 tools/analyze_anchor_timeline.py /tmp/bench_out/logs/.../multi_rate_*.csv

# 单元测试
g++ -std=c++17 -I. -I/path/to/MultiHarmonicAO/include tests/test_control.cpp \
  control/AssistStateMachine.cpp control/FreezeManager.cpp control/GaitFeatureExtractor.cpp \
  control/IntentDetector.cpp control/PhaseEstimator.cpp control/TorqueProfile.cpp \
  control/StopDetector.cpp control/StopTorqueLimiter.cpp -o /tmp/test_control && /tmp/test_control
python3 -m unittest tests.test_analyze_run tests.test_startup_signal_diagnostics -q
```

---

## 后续路线图（建议）

1. **实机 / 半实物：** `stop_go`、`repeated_stop_go` — 硬门槛 `anchor_update_during_stop_count = 0`
2. **V2.3：** stride segmentation **只记日志**，不参与控制
3. **暂缓：** `anchor_phase_gain`、完整双层 AO（需 stride 切分稳定）
