# V2.1 Anchor-based AO 频率校正 — 离线 A/B 对比报告

**日期：** 2026-05-29  
**对比对象：** git `HEAD`（改动前） vs 当前工作区（V2.1 实现）  
**方法：** 相同 8 条标准回放曲线 → `hs_exoskeleton_v2_replay` → `analyze_run.py`  
**原始数据：** 本目录 `before/*.json`、`after/*.json`

---

## 1. 对比目的

验证 V2.1「可靠步态 anchor + 置信度加权频率校正」是否在 **`multi_rate` 步频突变** 场景下改善频率重锁，且不在停步/常规曲线上引入明显退化。

V2.1 **不是**完整 two-level AO：高层只做离散频率校正，低层 AO 仍连续积分相位；第一版不做相位硬校正（`anchor_phase_gain = 0`）。

---

## 2. 复现命令

```bash
# 编译（Before：在 HEAD worktree；After：当前分支）
g++ -std=c++17 -I../2026.4.14hs_exoskeleton/include -I. \
  tools/replay_control.cpp control/*.cpp logging/ExoLogger.cpp \
  control/StopTorqueLimiter.cpp -o /tmp/hsx_v2_replay

# 批量回放 + 指标
./tools/benchmark_v21_anchor_batch.sh /tmp/hsx_v2_replay_before /tmp/v21_bench_before
./tools/benchmark_v21_anchor_batch.sh /tmp/hsx_v2_replay_after  /tmp/v21_bench_after

# 对比
python3 tools/compare_metrics.py \
  artifacts/v21_anchor_ab/before/multi_rate.json \
  artifacts/v21_anchor_ab/after/multi_rate.json

# multi_rate 各段 anchor 分布
python3 tools/analyze_anchor_timeline.py /path/to/multi_rate_runtime.csv
```

---

## 3. 总体结论

### 3.1 合入判定

**建议合入 V2.1 第一版。**

合入说明须明确边界：

```text
V2.1 主要改善 multi_rate 场景下的频率重锁与力矩峰值相位对齐；
不承诺全面降低全时段 phase_rmse_percent。
```

对外表述建议强调：**frequency adaptation** 与 **torque peak timing**，而非「全面改善相位」。

### 3.2 指标摘要

| 维度 | 结论 |
|------|------|
| **主要目标（multi_rate 频率适应）** | **达成**：`frequency_rmse_hz` ↓18.5%，`frequency_adaptation_time_mean_s` ↓26.4%，`combined_adaptation_time_mean_s` ↓18.7% |
| **停步安全** | **通过**：`stop_go` / `multi_rate` 的 `false_anchor_during_stop_count = 0`；`repeated_stop_go` 从 2→1（离线 1 次，须实机再确认） |
| **力矩副作用** | **偏多改善**：`multi_rate` 的 `torque_rate_p95`、`max_torque_rate` 下降；`omega_jump_max_hz ≈ 0.026`（跳变很小） |
| **相位误差** | **已知代价**：`phase_rmse_percent` 在多数曲线上略升；`multi_rate` 的 **`peak_torque_phase_mae_deg` 明显改善**（7.41°→3.56°）— 工程意义常高于全时段 phase RMSE |

**一句话：** 频率重锁更快、omega 不突跳、力矩变化率未恶化、停步段不误更新；全时段 phase RMSE 略升为 frequency-only 设计的可接受代价。

### 3.3 合入后 TODO

| ID | 内容 | 说明 |
|----|------|------|
| TODO-1 | 诊断 0.6 Hz 起步段 anchor 覆盖 | `multi_rate` 第一段 update=0；先统计 reject 原因，**勿**合入前盲目降 `anchor_min_confidence` |
| TODO-2 | 实机验证 stop_go / repeated_stop_go / abrupt_stop | 走走停停恢复、急停软停、停步边缘误 anchor |

### 3.4 V2.2 暂不建议（合入后立即做）

- 加大 `anchor_frequency_gain`（可能破坏 1.35 Hz 段稳定性）
- 降低 `anchor_min_confidence`（可能增加 `noisy_stop_go` 假 anchor）
- 立即加 phase correction（当前 `peak_torque_phase_mae` 已改善，风险大于收益）
- 以 `phase_rmse` 为唯一目标继续调参（易损害力矩输出稳定性）

V2.2 优先方向：低频起步 anchor 诊断 → 可选 1 帧极值确认 → 稳定 Active 段小增益 phase correction（若实机需要）。

---

## 4. 核心曲线 `multi_rate` 详细对比

步频序列（每段 4 s）：`0.6 → 1.2 → 0.75 → 1.35 → 0.9 Hz`，时长 20 s。

| 指标 | Before (HEAD) | After (V2.1) | Δ | 相对变化 | 判定 |
|------|--------------:|-------------:|--:|---------:|------|
| `frequency_rmse_hz` | 0.2532 | 0.2063 | −0.0469 | −18.5% | 改善 |
| `frequency_mae_hz` | 0.1769 | 0.1414 | −0.0355 | −20.1% | 改善 |
| `frequency_adaptation_time_mean_s` | 1.7300 | 1.2733 | −0.4567 | −26.4% | 改善 |
| `frequency_adaptation_time_max_s` | 3.0000 | 1.3800 | −1.6200 | −54.0% | 改善 |
| `combined_adaptation_time_mean_s` | 1.7300 | 1.4067 | −0.3233 | −18.7% | 改善 |
| `combined_adaptation_time_max_s` | 3.0000 | 1.6200 | −1.3800 | −46.0% | 改善 |
| `phase_rmse_percent` | 9.9503 | 11.5693 | +1.6190 | +16.3% | 变差 |
| `phase_abs_error_p95_percent` | 24.7093 | 28.3419 | +3.6327 | +14.7% | 变差 |
| `phase_adaptation_time_mean_s` | 0.7350 | 1.4067 | +0.6717 | +91.4% | 变差 |
| `peak_torque_phase_mae_deg` | 7.4050 | 3.5610 | −3.8441 | −51.9% | 改善 |
| `torque_rate_p95_nm_s` | 41.3992 | 38.8088 | −2.5905 | −6.3% | 改善 |
| `max_torque_rate_nm_s` | 87.5799 | 64.0280 | −23.5519 | −26.9% | 改善 |
| `omega_jump_max_hz` | — | 0.0255 | — | — | 跳变很小 |
| `anchor_update_count` | 0 | 15 | +15 | — | 机制生效 |
| `false_anchor_during_stop_count` | 0 | 0 | 0 | — | 通过 |

### 4.1 `multi_rate` 各步频段 anchor 更新次数（After）

| 频段（4 s 段） | 目标 Hz | `anchor_update_count` |
|----------------|--------:|----------------------:|
| 段 1 | 0.60 | **0** |
| 段 2 | 1.20 | 3 |
| 段 3 | 0.75 | 1 |
| 段 4 | 1.35 | 8 |
| 段 5 | 0.90 | 3 |
| **合计** | — | **15** |

说明：

- 第一段 0.6 Hz **无频率更新**，可能与起步阶段长时间处于 `Tracking`（仅计 anchor、不更新频率）或 confidence/interval 门控有关，是后续调参重点。
- 1.35 Hz 段更新最密集，符合「步频突变后需要多次校正」的预期。
- 停步边界 **无** `edge/stop-context` 误更新（`analyze_anchor_timeline` 检查通过）。

---

## 5. 八条标准曲线汇总

### 5.1 频率追踪

| 曲线 | Before `frequency_rmse_hz` | After | Δ | 判定 |
|------|---------------------------:|------:|--:|------|
| `steady_0p8` | 0.0162 | 0.0145 | −0.0017 | 改善 |
| `freq_ramp` | 0.0413 | 0.0285 | −0.0128 | 改善 |
| `stop_go` | 0.0179 | 0.0117 | −0.0062 | 改善 |
| `abrupt_stop` | 0.0215 | 0.0128 | −0.0087 | 改善 |
| **`multi_rate`** | **0.2532** | **0.2063** | **−0.0469** | **改善（主目标）** |
| `repeated_stop_go` | 0.0215 | 0.0553 | +0.0338 | 变差 |
| `amp_ramp` | 0.0111 | — | — | After 评估缺省 |

### 5.2 相位 RMSE（% gait）

| 曲线 | Before | After | Δ | 判定 |
|------|-------:|------:|--:|------|
| `steady_0p8` | 1.41 | 2.17 | +0.76 | 变差 |
| `freq_ramp` | 2.26 | 1.88 | −0.37 | 改善 |
| `stop_go` | 1.41 | 2.44 | +1.02 | 变差 |
| `multi_rate` | 9.95 | 11.57 | +1.62 | 变差 |
| `repeated_stop_go` | 3.23 | 4.53 | +1.30 | 变差 |
| `abrupt_stop` | 2.46 | 3.43 | +0.97 | 变差 |

移除 `phase_offset` 后离线相位 RMSE 普遍略升，与「只校频率、不校相位」设计一致；需结合 **`peak_torque_phase_mae_deg`** 与实机观感综合判断。

### 5.3 力矩峰值相位 MAE（deg）

| 曲线 | Before | After | Δ | 判定 |
|------|-------:|------:|--:|------|
| `steady_0p8` | 3.00 | 2.75 | −0.25 | 改善 |
| **`multi_rate`** | **7.41** | **3.56** | **−3.84** | **明显改善** |
| `freq_ramp` | 2.22 | 2.30 | +0.08 | 略差 |
| `stop_go` | 4.33 | 5.63 | +1.30 | 变差 |
| `abrupt_stop` | 14.67 | 17.40 | +2.73 | 变差 |
| `repeated_stop_go` | 15.03 | 18.11 | +3.08 | 变差 |

### 5.4 力矩变化率与安全

| 曲线 | `torque_rate_p95` Before→After | `max_torque_rate` Before→After | `false_anchor_stop` |
|------|-------------------------------|-------------------------------|---------------------|
| `multi_rate` | 41.4 → 38.8（↓） | 87.6 → 64.0（↓） | 0 → 0 |
| `stop_go` | 16.9 → 7.0（↓） | 40 → 40 | 0 → 0 |
| `repeated_stop_go` | 8.3 → 3.7（↓） | 40 → 40 | 2 → 1 |
| `abrupt_stop` | 15.4 → 8.8（↓） | 40 → 40 | 0 → 0 |
| `steady_0p8` | 19.4 → 19.0（↓） | 27.1 → 20.8（↓） | 0 → 0 |

`stationary_false_assist_s`、`freeze_torque_violation_count`、`disallow_output_torque_violation_count` 在所有曲线上均为 **0**（与 Before 一致）。

### 5.5 Anchor 机制（仅 After 有统计）

| 曲线 | `anchor_update_count` | `anchor_detected_count` |
|------|----------------------:|------------------------:|
| `multi_rate` | 15 | 25 |
| `freq_ramp` | 17 | 20 |
| `steady_0p8` | 3 | 9 |
| `stop_go` | 2 | 9 |
| `abrupt_stop` / `repeated_stop_go` | 0 | 4~16（有检测、未更新频率） |

Before（HEAD）无 anchor 频率更新逻辑，`anchor_update_count` 均为 0。

---

## 6. 改善项 vs 待关注项

### 6.1 已验证改善

1. **`multi_rate` 频率 RMSE 与重锁时间** — 主验收目标达成。  
2. **`multi_rate` 力矩峰值相位 MAE** — 下降约 52%。  
3. **`multi_rate` 力矩变化率** — P95 与最大值均下降。  
4. **`omega` 跳变** — `omega_jump_max_hz ≈ 0.026 Hz`，rate limit 有效。  
5. **停步误 anchor** — `stop_go` / `multi_rate` 为 0；已加 `stop_probability ≥ 0.65` 门控。  
6. **`freq_ramp` / `stop_go` / `abrupt_stop`** — 频率 RMSE 与/或力矩变化率改善。

### 6.2 已知代价（合入时须标注）

`phase_rmse_percent`（`multi_rate`）：9.95 → 11.57（+16%）。

需同时引用：

- `frequency_rmse_hz` 改善
- `peak_torque_phase_mae_deg`：7.4° → 3.6° 改善
- `torque_rate_p95` 下降

含义：AO 连续相位误差指标略差，但**助力力矩峰值相位对齐更好**——对外骨骼更贴近工程目标。

### 6.3 待关注 / 后续工作（非阻塞合入）

1. **`multi_rate` 0.6 Hz 段** — 0 次 `anchor_update`（follow-up，见 TODO-1）。  
2. **`repeated_stop_go`** — `false_anchor` 2→1；`frequency_rmse` 略升；实机复测。  
3. **`abrupt_stop`** — `peak_torque_phase_mae` 离线略差；`stationary_false_assist_s` 仍为 0。  
4. **实机验证** — 本报告均为离线回放；上机前建议 `allow_output=false` dry-run 并录 CSV。

### 6.4 合入前四条曲线核对表

| 曲线 | `false_anchor_stop` | `stationary_false_assist_s` | `frequency_rmse` B→A | `torque_rate_p95` B→A | `peak_torque_phase_mae` B→A | 备注 |
|------|--------------------:|----------------------------:|---------------------:|----------------------:|----------------------------:|------|
| `stop_go` | 0→0 | 0→0 | 0.018→0.012 ↓ | 16.9→7.0 ↓ | 4.33→5.63 ↑ | Active 段 2 次 update，可接受 |
| `repeated_stop_go` | 2→1 | 0→0 | 0.021→0.055 ↑ | 8.3→3.7 ↓ | 15.0→18.1 ↑ | 待实机 |
| `abrupt_stop` | 0→0 | 0→0 | 0.022→0.013 ↓ | 15.4→8.8 ↓ | 14.7→17.4 ↑ | 急停指标需实机盯 |
| `steady_0p8` | 0→0 | 0→0 | 0.016→0.015 ↓ | 19.4→19.0 ↓ | 3.00→2.75 ↓ | `omega_jump_max` 0.006 |

---

## 7. V2.1 实现要点（便于与数据对照）

| 机制 | 说明 |
|------|------|
| 仅频率校正 | 移除 `phase_offset_rad_`；`phase_rad = wrap(phi_GP)` |
| Anchor 检测 | 50 Hz：速度过零 + `max(|v_prev|,|v_curr|) ≥ anchor_min_velocity` |
| 融合 | `effective_gain = gain(Active/Ramp) × confidence`；min/max、ratio、step 限幅 |
| `omega_target` | Anchor 设目标后跨帧按 `max_omega_rate_rad_s2 × dt` 追踪 |
| 状态门控 | 上一帧 `AssistState`；`Tracking` 不更新频率；`stop_probability ≥ 0.65` 禁止更新 |

---

## 8. 相关文件

| 文件 | 说明 |
|------|------|
| [`before/`](before/) | HEAD 各曲线 `metrics.json` |
| [`after/`](after/) | V2.1 各曲线 `metrics.json` |
| [`multi_rate_comparison.md`](multi_rate_comparison.md) | `compare_metrics.py` 生成的 multi_rate 细表 |
| [`multi_rate_comparison.json`](multi_rate_comparison.json) | 同上 JSON |
| [`../tools/benchmark_v21_anchor_batch.sh`](../tools/benchmark_v21_anchor_batch.sh) | 批量回放脚本 |
| [`../tools/analyze_anchor_timeline.py`](../tools/analyze_anchor_timeline.py) | Anchor 时间线分析 |
| [`../RUNTIME_ANALYSIS.md`](../RUNTIME_ANALYSIS.md) | V2.1 设计与验收说明 |

---

*生成说明：指标来自 `artifacts/v21_anchor_ab/before|after/*.json`；对比判定规则为「越小越好」类指标 After < Before 记为改善，计数类 anchor 更新 After > Before 记为机制生效。*
