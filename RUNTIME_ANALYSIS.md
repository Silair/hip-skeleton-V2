# hs_exoskeleton_v2 运行数据记录与离线分析

## 运行时记录

V2 每圈控制会通过 `logging/ExoLogger` 写 CSV。新增字段覆盖 AO 相位评估和控制安全指标所需数据：

- 相位/频率：`Phase`, `PhaseValid`, `AnchorDetected`, `Frequency`, `Amplitude`, `AoSignalEstimateRad`, `AoSignalErrorRad`
- 步态信号：`PhaseSignalRad`, `FilteredPhaseSignalRad`, `SpreadDeg`, `PhaseVelocityDegS`, `SignedPhaseVelocityDegS`
- 意图/冻结/助力：`MotionConfidence`, `StopProbability`, `FreezeRequested`, `PhaseTrackingEnabled`, `RecoveryActive`, `AssistState`, `TorqueScale`, `AllowOutput`
- 关节/电机：`LeftJointPos`, `RightJointPos`, `LeftJointVel`, `RightJointVel`, `LeftRawMotorPos`, `RightRawMotorPos`, `LeftRawMotorVel`, `RightRawMotorVel`
- 输出：`LeftTorqueCmd`, `RightTorqueCmd`

程序结束后会打印日志路径和分析命令，例如：

```bash
python3 hs_exoskeleton_v2/tools/analyze_run.py Data/2026.05.28/AO_v2_test_1_2026.05.28.csv
```

## 生成报告

```bash
python3 hs_exoskeleton_v2/tools/analyze_run.py <runtime.csv>
```

可选参数：

```bash
python3 hs_exoskeleton_v2/tools/analyze_run.py <runtime.csv> \
  --output-dir <report-dir> \
  --peak-min-spread-deg 18 \
  --lead-angle-rad 0.20
```

输出：

- `metrics.json`：指标摘要
- `enriched_timeseries.csv`：补充 `ReferencePhase`, `ReferenceFrequency`, `PhaseErrorDeg`, `PhaseErrorPercent` 的时序表
- `index.html`：可直接浏览的 SVG 图表报告

## 当前离线指标

分析脚本会从 `FilteredPhaseSignalRad` 和 `SignedPhaseVelocityDegS` 的峰/谷事件构造参考相位：

- 正峰：参考相位 `π/2`
- 负峰：参考相位 `3π/2`
- 相邻峰/谷之间线性插值

计算指标：

- `phase_rmse_percent`：AO 相位相对参考相位的 RMSE，占 gait cycle 百分比
- `phase_mae_percent`、`phase_abs_error_p95_percent`
- `frequency_rmse_hz`、`frequency_mae_hz`
- `ao_reconstruction_rmse_rad`
- `convergence_time_s_at_5_percent`
- `frequency_transition_count`：参考步频发生阶跃变化的次数
- `frequency_adaptation_time_mean_s` / `frequency_adaptation_time_max_s`：步频突变后 AO 频率误差重新低于 `0.10 Hz` 的平均/最大耗时
- `phase_adaptation_time_mean_s` / `phase_adaptation_time_max_s`：步频突变后 AO 相位误差重新低于 `5% gait cycle` 的平均/最大耗时
- `combined_adaptation_time_mean_s` / `combined_adaptation_time_max_s`：频率和相位都重新达标所需耗时
- `frequency_transition_windows`：每次步频突变的时间、前后频率、频率/相位重新锁定耗时明细
- `max_abs_torque_nm`
- `max_torque_rate_nm_s`、`torque_rate_p95_nm_s`
- `freeze_torque_violation_count`
- `disallow_output_torque_violation_count`
- `stationary_false_assist_s`
- `simultaneous_high_torque_ratio`
- `peak_torque_phase_mae_deg`

## 上实机前建议阈值

先用合成数据、旧日志回放和干跑模式检查：

```text
steady phase RMSE < 5%
transition phase RMSE < 8~10%
frequency RMSE < 0.05~0.10 Hz
frequency/phase relock after cadence step <= 0.5~1.0 s
convergence <= 3 gait cycles
peak torque phase error < 10~15°
torque never exceeds configured limit
freeze/fault/disallow-output torque always zero
stationary false assist duration < 0.5 s
```


## 控制效果评价判定

`tools/analyze_run.py` 会在生成指标后执行离线控制效果评价，默认输出：

- `evaluation.json`：结构化 PASS/FAIL 判定、三组检查结果、阈值和失败项。
- `index.html`：新增“控制效果判定”区，显示总评、分组状态、逐项阈值对比和免责声明。
- CLI 摘要：打印 `evaluation_status: PASS|FAIL`。

评价分三组：

1. **相位/频率跟踪**：参考相位事件数、相位 RMSE、相位 P95、频率 RMSE、AO 重构误差、收敛时间。
2. **助力力矩效果**：最大绝对力矩、最大力矩变化率、双侧同时高力矩比例、峰值力矩相位误差。
3. **安全门控**：冻结时非零力矩、禁止输出时非零力矩、静止误助力时长。

默认阈值可通过 CLI 覆盖：

```bash
python3 hs_exoskeleton_v2/tools/analyze_run.py <runtime.csv> \
  --phase-rmse-threshold-percent 5 \
  --phase-p95-threshold-percent 10 \
  --frequency-rmse-threshold-hz 0.10 \
  --frequency-adaptation-threshold-s 1.0 \
  --phase-adaptation-threshold-s 1.0 \
  --combined-adaptation-threshold-s 1.0 \
  --max-torque-nm 8 \
  --max-torque-rate-nm-s 80 \
  --stationary-false-assist-threshold-s 0.5 \
  --peak-torque-phase-threshold-deg 15 \
  --min-reference-events 4
```

默认情况下，评价 `FAIL` 仍返回 exit 0，以便手动实验时完整生成报告。自动化批处理可加：

```bash
python3 hs_exoskeleton_v2/tools/analyze_run.py <runtime.csv> --fail-on-evaluation
```

此时评价 `FAIL` 返回 exit 2；CSV 读取/脚本错误仍按普通异常失败。

> 注意：本评价仅用于离线控制日志质量检查。PASS 表示当前日志中的控制指标满足配置阈值，不代表临床安全性、人体实验许可或硬件实际输出验证。参考相位仍来自峰/谷事件构造的近似参考，不等价于外部 ground truth。

## 改进前/改进后对比

针对双层 AO、anchor frequency update 这类“更快适应步频突变”的改动，推荐固定同一条回放曲线，分别保存改动前后的 `metrics.json`，再比较关键指标：

```bash
python3 hs_exoskeleton_v2/tools/compare_metrics.py \
  /tmp/before_multi_rate/metrics.json \
  /tmp/after_multi_rate/metrics.json \
  --markdown-output /tmp/multi_rate_comparison.md \
  --json-output /tmp/multi_rate_comparison.json
```

优先看这些指标是否下降：

- `frequency_adaptation_time_mean_s` / `frequency_adaptation_time_max_s`
- `phase_adaptation_time_mean_s` / `phase_adaptation_time_max_s`
- `combined_adaptation_time_mean_s` / `combined_adaptation_time_max_s`
- `frequency_rmse_hz`
- `phase_rmse_percent`
- `phase_abs_error_p95_percent`
- `max_torque_rate_nm_s`

解释口径：

- 如果 `frequency_rmse_hz` 下降但 `phase_adaptation_time_*` 不降，说明频率估计更准了，但相位还没有更快锁定；
- 如果 `combined_adaptation_time_*` 下降，同时稳态曲线 `steady_0p8`、`freq_ramp` 不变差，才算真正改善了步频切换适应；
- 如果 `max_torque_rate_nm_s` 上升明显，说明频率快速融合可能让力矩变化更急，需要降低融合增益或加限斜率。

## 注意

- `ReferencePhase` 是由步态信号峰/谷离线构造的近似参考，不等价于足底压力、IMU 或动作捕捉 ground truth。
- 如果后续有 heel strike/toe off 或动作捕捉数据，应优先用外部 gait events 生成参考相位。
- 离线指标通过后，建议先上实机 dry-run：读取真实电机角度、计算 V2 输出并记录日志，但强制 `allow_output=false` 或发送 0 torque。

## 与离线回放配合使用

如果还没有实机 runtime CSV，可以先使用 `OFFLINE_REPLAY.md` 中的曲线生成器和 `hs_exoskeleton_v2_replay` 离线生成同结构日志，再运行本分析脚本。这样可以在上实机前检查 AO 相位误差、频率追踪、助力峰值相位、冻结/禁止输出违规等指标。


## V2.1 Anchor-based Gait AO Frequency Correction（已实现）

实现位置：`control/PhaseEstimator.cpp`、`config/ControlConfig.h`（`PhaseConfig` anchor 字段）、`logging/ExoLogger.cpp`（Anchor* 列）、`tools/analyze_run.py`（anchor 指标）。

实现要点（第一版）：

- **不是**完整复刻 gait-based AO / two-level AO：高层只做离散 **anchor-based frequency correction**；低层 AO 仍负责连续相位积分；不做 stride template、幅值学习、相位硬校正。
- 低层 AO 连续积分相位；可靠 Peak/Valley anchor 仅做 **频率** 离散校正（`anchor_phase_gain = 0`，已移除 `phase_offset_rad_`）。
- 异类相邻 anchor 用 `0.5 / dt_anchor` 估计半周期频率；confidence 加权融合 + min/max、ratio、step。**Anchor 事件只更新 `omega_target`；`oscillator_.omega` 在后续控制周期内按 `max_omega_rate_rad_s2 * dt` 限速追踪目标**（避免在 anchor 瞬间直接写 `omega` 造成突跳）。
- `PhaseEstimator::update` 使用 **上一帧** `AssistState` 与 `stop_probability`（`>= anchor_update_disable_stop_probability` 时禁止更新，默认 0.65）。
- 50 Hz anchor：速度过零 + `max(|v_prev|, |v_curr|) >= anchor_min_velocity` + spread/refractory。

离线 HEAD vs V2.1 对比（`tools/benchmark_v21_anchor_batch.sh` + `compare_metrics.py`，2026-05-29）；完整数据与改善分析见 [`artifacts/v21_anchor_ab/V21_ANCHOR_AB_REPORT.md`](artifacts/v21_anchor_ab/V21_ANCHOR_AB_REPORT.md)。

| 曲线 | 主要变化 |
| --- | --- |
| `multi_rate` | `frequency_rmse_hz` 0.253→0.208（-18%）；`frequency_adaptation_time_mean_s` 1.73→1.29s；`combined` 改善；`phase_rmse_percent` 9.95→11.60（变差，需继续盯 `peak_torque_phase_mae`） |
| `stop_go` / `freq_ramp` | `false_anchor_during_stop=0`；频率 RMSE 略降 |
| `steady_0p8` | `peak_torque_phase_mae` 未单独退化（整体 phase_rmse 略升） |

Anchor 时间线：`python3 tools/analyze_anchor_timeline.py <runtime.csv>` 可按 `multi_rate` 各 4s 段统计 `anchor_update_count`。

### V2.2 Anchor 诊断、延迟确认与 Tracking 延迟频率（已实现）

第一版 V2.2 聚焦 **诊断 + 抗噪 + 低频起步覆盖**，暂不默认开启相位校正：

- `AnchorRejectReason` 枚举写入 `PhaseEstimate::anchor_reject_reason` 与 runtime CSV 列 `AnchorRejectReason`。
- `analyze_run.py` 输出 `anchor_reject_reason_counts`；`analyze_anchor_timeline.py` 按 `multi_rate` 各 4s 段输出 `segment_diagnostics`（候选/检测/更新/拒绝原因）。
- `anchor_confirm_delay_frames = 1`（50 Hz 下约 20 ms）：过零候选延迟 1 帧确认；失败记 `ConfirmFailed`。
- `enable_tracking_deferred_frequency`：Tracking 段测得可靠半周期后缓存，进入 Ramp/Active 首帧应用一次（缓解 0.6 Hz 段仅 Tracking 时 `anchor_update_count=0`）。
- CSV 列 `AnchorCandidate`：记录延迟确认前的可靠过零候选。

停步验收拆分为：`anchor_update_during_stop_count`（硬门槛 = 0）、`anchor_rejected_during_stop_count`、`anchor_candidate_during_stop_count`（信息性）。勿用旧版混合 `false_anchor_during_stop_count` 判断安全性。

0.6 Hz 起步：`python3 tools/startup_signal_diagnostics.py <multi_rate.csv>`。

### V2.2.1 Startup frequency prior（已实现）

独立于稳态 anchor，仅在 **Tracking/Ramp** 检测起步趋势（`stop_probability < 0.55`、`motion_confidence >= 0.48`、同向速度连续 N 帧、spread 增加），估计 `startup_frequency_prior_hz`，在 **Tracking/Ramp** 中用 `startup_prior_gain`（默认 0.20；Tracking 默认再乘 `startup_prior_tracking_gain_scale = 0.50`）混合 `omega_target`（仍受 `max_omega_rate_rad_s2` 限速）。

CSV：`StartupPriorValid/Candidate/Applied/FrequencyHz/Confidence`。指标：`startup_prior_candidate_count`、`startup_prior_applied_count`、`startup_prior_frequency_hz_mean`。

收口验证见 [`artifacts/v221_startup_prior/V221_STARTUP_PRIOR_REPORT.md`](artifacts/v221_startup_prior/V221_STARTUP_PRIOR_REPORT.md)。`multi_rate` 当前结果：

- `frequency_rmse_hz`：V2.2 0.2058 → V2.2.1 0.2035；
- `combined_adaptation_time_mean_s`：1.4067 → 1.3667；
- `phase_rmse_percent`：11.5610 → 11.0838；
- `startup_prior_applied_count = 1`，`startup_prior_frequency_hz_mean ≈ 0.6098 Hz`；
- `anchor_update_during_stop_count = 0`。

仍属 V2.3+：stride segmentation 日志版；**不要**马上上 `anchor_phase_gain` 或完整双层 AO。

### 合入评审结论（V2.1 第一版）

**建议合入。** 定位与边界须写清楚：

```text
V2.1 主要改善 multi_rate（及类似步频突变）下的频率重锁与力矩峰值相位对齐；
不承诺全面降低全时段 phase_rmse_percent。
```

V2.1 Anchor-based Frequency Correction 已完成第一版实现。该版本不是完整 two-level AO，而是在现有 AO 上增加可靠 Peak/Valley anchor 事件层，用于对 `omega_target` 做受限频率校正，并通过每帧 omega rate limit 平滑追踪目标频率。

A/B 结果（`multi_rate`，HEAD → V2.1）：

- `frequency_rmse_hz`：0.253 → 0.206（约 −18%）
- `frequency_adaptation_time_mean_s`：1.73 s → 1.27 s（约 −26%）
- `combined_adaptation_time_mean_s`：1.73 s → 1.41 s（约 −19%）
- `torque_rate_p95_nm_s`：41.4 → 38.8
- `omega_jump_max_hz`：0.026（频率更新无明显突跳）
- `false_anchor_during_stop`：0
- `peak_torque_phase_mae_deg`：7.4° → 3.6°（力矩峰值相位对齐改善）

已知代价：`phase_rmse_percent` 9.95 → 11.57。第一版只做频率校正、不做相位 offset / phase correction，该现象可接受；工程上更应关注 **frequency adaptation** 与 **torque peak timing**，而非单独优化全时段 `phase_rmse`。

合入后 TODO：

1. **诊断 0.6 Hz 低频起步段 anchor 覆盖不足**（`multi_rate` 第一段 `anchor_update_count = 0`）：先加 reject 原因统计，勿在 V2.1 合入前盲目降低 `anchor_min_confidence` 或加大 `anchor_frequency_gain`。
2. **实机验证** `stop_go` / `repeated_stop_go` / `abrupt_stop`：走走停停恢复、急停软停边界、是否存在误 anchor 或恢复偏慢。

#### 合入前四条曲线核对（离线回放，2026-05-29）

| 曲线 | 核对项 | Before → After | 结论 |
| --- | --- | --- | --- |
| `stop_go` | `false_anchor_during_stop` | 0 → 0 | 通过 |
| | `stationary_false_assist_s` | 0 → 0 | 通过 |
| | `frequency_rmse_hz` | 0.018 → 0.012 | 改善 |
| | `torque_rate_p95` | 16.9 → 7.0 | 改善 |
| | `anchor_update_count` | 0 → 2（Active 段） | 可接受 |
| `repeated_stop_go` | `false_anchor_during_stop` | 2 → **1** | 待实机确认（离线 1 次需排查） |
| | `stationary_false_assist_s` | 0 → 0 | 通过 |
| | `torque_rate_p95` | 8.3 → 3.7 | 改善 |
| | `anchor_update_count` | 0 → 0 | 无频率更新（门控有效） |
| `abrupt_stop` | `stationary_false_assist_s` | 0 → 0 | 通过 |
| | `torque_rate_p95` | 15.4 → 8.8 | 改善 |
| | `peak_torque_phase_mae_deg` | 14.7 → 17.4 | 离线略差，合入后实机盯 |
| | `false_anchor_during_stop` | 0 → 0 | 通过 |
| `steady_0p8` | `phase_rmse_percent` | 1.41 → 2.17 | 略升（已知代价） |
| | `peak_torque_phase_mae_deg` | 3.00 → 2.75 | 略改善 |
| | `omega_jump_max_hz` | — → 0.006 | 抖动很小 |
| | `anchor_update_count` | 0 → 3 | 合理 |

---

### 定位（设计说明）

严格来说，V2.1 不应称为完整“双层 AO”复现。更准确的定位是：

```text
低层 AO 连续积分相位
+
高层 gait anchor 事件离散校正频率
```

也就是 **lightweight gait-based AO / anchor-based frequency correction**。

它受 **Effective Prediction of Gait Phase for Assisted Walking by Means of Gait-Based Adaptive Oscillators**（IEEE T-ASE 2025, DOI: `10.1109/TASE.2024.3520148`）中 two-level AO 思路启发，但暂不完整复刻论文的高层 stride learning、周期模板或幅值/相位形状学习。

推荐表述：

> 受 gait-based adaptive oscillator 中 two-level AO 思路启发，V2 暂不完整复刻其高层 stride learning，而是实现一个轻量化 anchor-based frequency correction。该方法保留当前单层 AO 的实时相位积分，同时利用可靠步态 anchor 对频率状态进行离散校正，以改善步频突变下的重新锁相速度。

### 背景

严格离线曲线 `multi_rate` 暴露出当前 AO 的一个真实短板：

```text
0.6 Hz -> 1.2 Hz -> 0.75 Hz -> 1.35 Hz -> 0.9 Hz
```

这类分段突变步频会让当前单层 AO 来不及重新锁相，表现为：

- `phase_rmse_percent` 升高；
- `phase_abs_error_p95_percent` 升高；
- `frequency_rmse_hz` 升高；
- `frequency_adaptation_time_mean_s` / `combined_adaptation_time_mean_s` 偏大；
- 频率变化导致力矩变化率接近或超过阈值。

根因链条：

```text
步频突变
↓
单层 AO 的 omega 适应滞后
↓
相位积分速度不匹配
↓
phase error 累积
↓
TorqueProfile 峰值时机偏移
↓
力矩变化率可能增大
```

因此优先不要调 `lead_angle`、不要改冻结/停步，而是在 `PhaseEstimator` 上补一个可靠 gait anchor 频率校正层。

### V2.1 核心方案

当前只有髋关节角度和角速度，没有足底压力、IMU 或力传感器。最可用的 gait event 是：

```text
髋角信号峰 / 谷 / 速度换向点
```

第一版只使用 Peak / Valley anchor：

```cpp
enum class AnchorType {
    Peak,
    Valley,
};
```

#### Anchor 类型与频率估计

不能无条件使用：

```cpp
measured_frequency_hz = 0.5 / dt_anchor;
```

该公式只适用于异类相邻 anchor：

```text
Peak -> Valley
Valley -> Peak
```

保守第一版建议：

```cpp
if (last.type != current.type) {
    measured_frequency_hz = 0.5 / dt_anchor;
} else {
    // Peak -> Peak 或 Valley -> Valley 可能来自漏检。
    // 第一版先不更新，避免半频误估。
    return;
}
```

后续如果要支持同类 anchor，则应使用：

```cpp
measured_frequency_hz = 1.0 / dt_anchor;
```

#### Anchor 可靠性条件

一个可靠 anchor 至少满足：

```text
角度幅值足够大
+
角速度发生明确换向
+
换向前后速度幅值足够大
+
距离上一个 anchor 的时间合理
+
当前处于允许更新的行走状态
```

伪代码：

```cpp
bool reliable_peak =
    prev_velocity_deg_s > anchor_min_velocity_deg_s &&
    curr_velocity_deg_s <= -anchor_min_velocity_deg_s &&
    std::abs(position_deg) > anchor_min_spread_deg;

bool reliable_valley =
    prev_velocity_deg_s < -anchor_min_velocity_deg_s &&
    curr_velocity_deg_s >= anchor_min_velocity_deg_s &&
    std::abs(position_deg) > anchor_min_spread_deg;
```

如果单点噪声明显，后续再加 30~50 ms 方向一致性确认或过零滞回。

#### Anchor confidence

不要让所有 anchor 都用同一个高增益。建议：

```cpp
effective_gain = anchor_frequency_gain * anchor_confidence;
```

confidence 可由三部分构成：

```cpp
double interval_ratio = dt_anchor / predicted_half_period;
double interval_score = clamp01(1.0 - std::abs(interval_ratio - 1.0) / 0.4);

double amplitude_score = clamp01((spread_deg - anchor_min_spread_deg) / anchor_spread_margin_deg);
double velocity_score = clamp01(anchor_velocity_abs_deg_s / anchor_velocity_reference_deg_s);

double anchor_confidence =
    0.4 * interval_score +
    0.3 * amplitude_score +
    0.3 * velocity_score;
```

只有：

```cpp
anchor_confidence >= anchor_min_confidence
```

才允许更新频率。

#### 频率更新限幅

即使 anchor 估计频率在合法范围内，也不能一次把 AO 频率拉飞。需要三层限制：

1. 绝对范围：

```cpp
measured_frequency_hz = clamp(measured_frequency_hz,
                              anchor_frequency_min_hz,
                              anchor_frequency_max_hz);
```

2. 相对比例限制：

```cpp
double max_ratio = anchor_max_frequency_ratio;  // e.g. 1.35
measured_frequency_hz = clamp(measured_frequency_hz,
                              current_frequency_hz / max_ratio,
                              current_frequency_hz * max_ratio);
```

3. 单次 step 限制：

```cpp
corrected_frequency_hz = moveToward(current_frequency_hz,
                                    measured_frequency_hz,
                                    anchor_max_frequency_step_hz);
```

最后再融合：

```cpp
double target_frequency_hz =
    (1.0 - effective_gain) * current_frequency_hz +
    effective_gain * corrected_frequency_hz;
```

工程上最好不要直接无约束写 `oscillator_.omega`，而是走目标频率/限速更新：

```cpp
omega_target = target_frequency_hz * 2.0 * M_PI;
oscillator_.omega = moveToward(oscillator_.omega,
                               omega_target,
                               max_omega_rate_rad_s2 * dt);
```

### 状态门控

`multi_rate` 是连续行走中的步频突变，适合 anchor 更新；`stop_go` / `noisy_stop_go` 含停止段，停止后第一个峰/谷可能只是姿态调整，不一定是真实步态周期。

状态门控建议：

| 状态 | Anchor 频率更新 |
| --- | --- |
| `Active` | 允许 |
| `RampUp` | 低增益允许 |
| `Reacquire` | 初期禁用，连续可靠 anchor 计数达到阈值后低增益/正常启用 |
| `Stopping` | 禁止 |
| `Frozen` | 禁止 |
| `Fault` | 禁止 |

第一版要求：

```text
Stopping / Frozen / Fault 禁止更新
刚恢复行走时等待 2 个可靠 anchor
Active walking 才允许正常增益更新
```

### 相位校正策略

第一版 **只校正频率，不校正相位**：

```cpp
anchor_phase_gain = 0.0;
```

原因：直接把 phase 重置到 anchor phase 容易造成 TorqueProfile 力矩突跳。

如果第一版后出现：

```text
frequency_rmse_hz 明显下降
但 phase_rmse_percent / phase_abs_error_p95_percent 仍偏高
```

第二版再加入小幅相位校正：

```cpp
double anchor_phase = expectedPhaseForAnchor(current_anchor.type);
double phase_error = wrapToPi(anchor_phase - oscillator_.phase);
phase_error = clamp(phase_error, -max_anchor_phase_step_rad, max_anchor_phase_step_rad);
oscillator_.phase += anchor_phase_gain * phase_error;
```

建议：

```text
anchor_phase_gain = 0.05 ~ 0.20
max_anchor_phase_step = 5° ~ 10°
```

但 anchor phase 的定义必须和当前 `left + right` 步态信号、`TorqueProfile` 相位约定重新标定，不能随意填。

### 推荐初版配置

```cpp
struct PhaseConfig {
    // existing AO limits
    double ao_min_frequency_hz = 0.45;
    double ao_max_frequency_hz = 1.60;

    // anchor frequency correction
    bool enable_anchor_frequency_update = true;
    double anchor_frequency_gain = 0.35;
    double anchor_phase_gain = 0.0;  // V2.1 第一版先不校正相位
    double anchor_frequency_min_hz = 0.45;
    double anchor_frequency_max_hz = 1.60;

    // anchor validation
    double anchor_min_interval_s = 0.18;
    double anchor_max_interval_s = 1.10;
    double anchor_min_spread_deg = 4.0;
    double anchor_min_velocity_deg_s = 8.0;
    double anchor_refractory_s = 0.12;

    // confidence / consistency gate
    double anchor_spread_margin_deg = 8.0;
    double anchor_velocity_reference_deg_s = 60.0;
    double anchor_min_confidence = 0.55;
    double anchor_max_frequency_ratio = 1.35;
    double anchor_max_frequency_step_hz = 0.20;
    double max_omega_rate_rad_s2 = 8.0;

    // state gating
    bool disable_anchor_update_during_stopping = true;
    bool disable_anchor_update_during_frozen = true;
    int reacquire_anchor_warmup_count = 2;
};
```

建议调参顺序：

```text
anchor_frequency_gain = 0.30 ~ 0.35
anchor_max_frequency_ratio = 1.30 ~ 1.40
anchor_max_frequency_step_hz = 0.15 ~ 0.20
anchor_phase_gain = 0.0
```

### 分阶段实现

#### 第一版：可靠 anchor + 受限频率校正

实现：

```text
检测 Peak/Valley anchor
↓
判断 anchor 是否可靠
↓
只用相邻异类 anchor 估计半周期频率
↓
对 measured_frequency 做 min/max、ratio limit、step limit
↓
按 confidence 加权融合到 omega target
↓
通过 omega rate limit 更新 oscillator_.omega
↓
Stopping/Frozen/Fault/Reacquire early 禁止更新
```

目标：

```text
降低 multi_rate 的 frequency_rmse_hz
降低 frequency_adaptation_time_mean_s / combined_adaptation_time_mean_s
不破坏 steady/freq_ramp/amp_ramp/stop_go/abrupt_stop
```

#### 第二版：小幅 anchor phase correction

仅当第一版后频率改善但相位指标仍差时执行：

```text
加入 anchor_phase_gain
限制单次 phase correction 不超过 5°~10°
监控 torque_rate_p95_nm_s 和 max_torque_rate_nm_s
```

#### 第三版：高层 stride memory

如果后续要更接近论文 gait-based AO，可再加入：

```text
上一 stride 周期
上一 stride 幅值
上一 stride 相位偏移
上一 stride 左右对称性
新 stride 初期的低层 AO 初始化
```

这不是当前 V2.1 的优先项。

### 新增评测 / 日志 TODO

已有指标：

- `frequency_transition_count`
- `frequency_adaptation_time_mean_s` / `frequency_adaptation_time_max_s`
- `phase_adaptation_time_mean_s` / `phase_adaptation_time_max_s`
- `combined_adaptation_time_mean_s` / `combined_adaptation_time_max_s`
- `frequency_transition_windows`

后续实现 anchor 更新时，还应在 runtime CSV / metrics 中补充：

| 指标 | 作用 |
| --- | --- |
| `anchor_update_count` | 实际执行高层频率更新的次数 |
| `rejected_anchor_count` | 被可靠性/状态门控拒绝的 anchor 次数 |
| `anchor_frequency_error_hz` | anchor 估计频率与离线参考频率的差 |
| `omega_jump_p95_hz` | 检查频率状态是否跳变过大 |
| `false_anchor_during_stop_count` | 检查停步段是否误触发 anchor |
| `phase_correction_deg_p95` | 若启用相位校正，监控相位跳变 |

其中 `false_anchor_during_stop_count = 0` 是 `stop_go` / `noisy_stop_go` 的关键验收项。

### 验收标准

`multi_rate` 应改善：

- `frequency_rmse_hz` 下降；
- `phase_rmse_percent` 下降或至少不恶化；
- `phase_abs_error_p95_percent` 下降或至少不恶化；
- `frequency_adaptation_time_mean_s` 下降；
- `combined_adaptation_time_mean_s` 下降；
- `torque_rate_p95_nm_s` 和 `max_torque_rate_nm_s` 不超过阈值。

不应破坏：

- `steady_0p8`；
- `freq_ramp`；
- `amp_ramp`；
- `stop_go`；
- `abrupt_stop`。

停步/噪声相关验收：

- `false_anchor_during_stop_count = 0`；
- `stop_go` 不出现停步段误助力；
- `noisy_stop_go` 若仍 FAIL，至少不能因 anchor update 进一步恶化，应检查 `rejected_anchor_count` 是否足够高、`anchor_update_count` 是否被状态门控限制。

### 风险与处理

- 噪声可能制造假 anchor，导致频率被错误快速更新：用 confidence、refractory、速度换向、interval gate 和状态门控抑制；
- 频率融合增益过大可能让 AO 抖动：先用 `anchor_frequency_gain = 0.30 ~ 0.35`，并启用 ratio/step/rate limit；
- 起步重捕获可能被误认为步频突变：Reacquire 初期至少等待 2 个可靠 anchor；
- 仅校正频率可能无法完全消除相位误差：先验证 frequency 指标，必要时第二版再小幅 phase correction；
- 直接写 `oscillator_.omega` 可能放大力矩变化率：通过 `omega_target` 和 `max_omega_rate_rad_s2` 限速更新。

### 当前结论

这个方向值得做，是当前 V2 里最合适的 AO 升级路径之一。但 V2.1 应避免“直接高增益更新 omega”，改成：

```text
可靠 anchor
+ confidence
+ 频率 min/max、ratio、step、rate limit
+ 状态门控
+ 第一版只校正频率
```

先证明 `multi_rate` 的频率重锁变快且常规曲线不退化，再决定是否进入小幅相位校正和更完整的 stride memory。
