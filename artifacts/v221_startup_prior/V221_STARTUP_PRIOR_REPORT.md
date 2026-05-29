# V2.2.1 Startup Frequency Prior — 收口验证报告

**日期：** 2026-05-29  
**验证对象：** 当前 HEAD `557f227`（V2.2.1）  
**基线：** V2.1 `artifacts/v21_anchor_ab/after/*.json`、V2.2 `artifacts/v22_anchor/*.json`  
**方法：** 标准离线回放套件 + `noisy_stop_go` 压力曲线 → `analyze_run.py` → artifacts 固化。

---

## 1. 结论

**V2.2.1 可以作为当前 anchor-frequency 分支的收口版本，但不建议继续加完整双层 AO。**

它相比 V2.2 的核心变化是：在 Tracking/Ramp 起步阶段加入 `StartupPrior`，用起步过零半周期估计一次低频先验，并以小增益混合到 `omega_target`。它不生成正式 anchor、不改相位、不绕过 omega rate limit。

`multi_rate` 收益：

| 指标 | V2.1 | V2.2 | V2.2.1 | 结论 |
| --- | ---: | ---: | ---: | --- |
| `frequency_rmse_hz` | 0.2063 | 0.2058 | **0.2035** | 小幅继续改善 |
| `frequency_adaptation_time_mean_s` | 1.2733 | **1.2400** | **1.2400** | 保持 V2.2 改善 |
| `combined_adaptation_time_mean_s` | 1.4067 | 1.4067 | **1.3667** | 小幅改善 |
| `phase_rmse_percent` | 11.5693 | 11.5610 | **11.0838** | 相位 RMSE 回落 |
| `phase_abs_error_p95_percent` | 28.3419 | 28.3419 | **26.8008** | P95 回落 |
| `peak_torque_phase_mae_deg` | 3.5610 | **3.4609** | 3.6172 | 略差但仍远优于 V2.0 |
| `max_torque_rate_nm_s` | 64.0280 | 64.5855 | **62.8895** | 改善 |
| `anchor_update_count` | 15 | 15 | 15 | 稳态 anchor 行为未被扰动 |
| `anchor_update_during_stop_count` | — | — | **0** | 安全硬门槛通过 |
| `startup_prior_applied_count` | — | — | **1** | 0.6 Hz 起步先验生效 |
| `startup_prior_frequency_hz_mean` | — | — | **0.6098** | 接近第一段真实 0.6 Hz |

解释：V2.2.1 没有改变稳态 anchor 更新次数；它只补上了 `multi_rate` 第一段 0.6 Hz 起步阶段的频率初始化，因此整体提升是小而稳的。

---

## 2. 完整曲线结果（当前 HEAD）

| 曲线 | Evaluation | `phase_rmse_percent` | `frequency_rmse_hz` | `peak_torque_phase_mae_deg` | `max_torque_rate_nm_s` | `anchor_update_during_stop_count` | `startup_prior_applied_count` |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `steady_0p8` | PASS | 2.1789 | 0.0147 | 2.7653 | 20.7804 | 0 | 1 |
| `freq_ramp` | PASS | 1.8856 | 0.0285 | 2.3054 | 42.5152 | 0 | 1 |
| `amp_ramp` | FAIL* | N/A | N/A | 3.1139 | 74.3595 | 0 | 1 |
| `stop_go` | PASS | 2.4398 | 0.0118 | 5.6350 | 40.0000 | 0 | 2 |
| `abrupt_stop` | FAIL* | 3.4691 | 0.0135 | 17.5332 | 40.0001 | 0 | 1 |
| `asymmetric` | FAIL* | N/A | N/A | 3.7719 | 20.2054 | 0 | 1 |
| `repeated_stop_go` | FAIL* | 3.9175 | 0.0677 | 17.2035 | 40.0001 | 0 | 6 |
| `multi_rate` | FAIL* | 11.0838 | 0.2035 | 3.6172 | 62.8895 | 0 | 1 |
| `noisy_stop_go` | FAIL* | 21.5897 | 6.2665 | 4.8692 | 56.6643 | 0 | 1 |

\* `FAIL` 为离线阈值判定，并不等价于控制安全失败。关键安全门槛 `anchor_update_during_stop_count = 0` 在所有曲线均通过。`amp_ramp/asymmetric` 的 `N/A` 主要来自参考峰/谷事件不足；`multi_rate/noisy_stop_go` 是压力测试，仍用于观察退化方向。

---

## 3. `multi_rate` 起步段诊断

`analyze_anchor_timeline.py`：

```text
anchor updates: 15
by segment: {'0.60Hz': 0, '1.20Hz': 3, '0.75Hz': 1, '1.35Hz': 8, '0.90Hz': 3}
edge/stop-context updates: none
```

`startup_signal_diagnostics.py` 对 0–4s 段：

```text
spread_deg_p95 ≈ 19.87
velocity_abs_p95 ≈ 74.86
zero_cross = 5
startup_prior_candidate = 22
startup_prior_applied = 1
startup_prior_frequency_hz_mean ≈ 0.6098
likely_causes = ['signal_ok_but_anchor_candidate_gate_failed']
```

含义：正式 anchor 仍因可靠过零/延迟确认等门控没有在 0.6 Hz 段更新；StartupPrior 用更宽松但一次性的起步趋势逻辑补了这个缺口，且估计频率接近真实 0.6 Hz。

---

## 4. 风险与限制

1. **V2.2.1 是小幅优化，不是质变。** `multi_rate frequency_rmse_hz` 从 V2.2 的 0.2058 降到 0.2035；提升稳定但有限。
2. **仍不建议启用 `anchor_phase_gain`。** 当前 `peak_torque_phase_mae` 已维持在较低水平；相位校正可能引入力矩突跳。
3. **`noisy_stop_go` 仍是压力失败项。** 但 `anchor_update_during_stop_count = 0`，说明停步段没有实际频率更新。
4. **`amp_ramp/asymmetric` 的参考事件不足仍会导致评价 N/A/FAIL。** 这是离线参考构造限制，后续应优先引入外部 gait event 或单独评价口径。

---

## 5. 下一步建议

1. **推送当前 V2.2.1。** 本地 ahead 的 3 个提交形成完整链条：V2.1 → V2.2 → V2.2.1。
2. **实机 dry-run。** `allow_output=false` 或 0 torque，录制真实 stop/go、repeated_stop_go、abrupt_stop。硬门槛：
   - `anchor_update_during_stop_count = 0`
   - `stationary_false_assist_s = 0`
   - `freeze_torque_violation_count = 0`
   - `disallow_output_torque_violation_count = 0`
3. **V2.3 只做 stride segmentation 日志版。** 先记录 stride 周期、幅值、左右对称性，不参与控制。
4. **暂缓完整双层 AO。** 等 stride 日志稳定并有实机证据后，再考虑高层 stride memory 或相位校正。

---

## 6. 复现命令

```bash
g++ -std=c++17 -I. -I/home/lin/code/2026.4.14hs_exoskeleton/include \
  tools/replay_control.cpp control/GaitFeatureExtractor.cpp control/PhaseEstimator.cpp \
  control/IntentDetector.cpp control/FreezeManager.cpp control/AssistStateMachine.cpp \
  control/TorqueProfile.cpp control/StopDetector.cpp control/StopTorqueLimiter.cpp \
  logging/ExoLogger.cpp logging/Clock.cpp -o /tmp/hsx_v221_replay

./tools/benchmark_v21_anchor_batch.sh /tmp/hsx_v221_replay /tmp/hsx_v221_bench

python3 tools/analyze_anchor_timeline.py /tmp/hsx_v221_bench/logs/.../multi_rate_*.csv \
  --json-output artifacts/v221_startup_prior/multi_rate_timeline.json

python3 tools/startup_signal_diagnostics.py /tmp/hsx_v221_bench/logs/.../multi_rate_*.csv \
  --json-output artifacts/v221_startup_prior/multi_rate_startup_signal.json
```
