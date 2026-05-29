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


## TODO：双层 AO / Gait-Based AO 用于步频突变适应

### 背景

严格离线曲线 `multi_rate` 暴露出当前 AO 的一个真实短板：

```text
0.6 Hz -> 1.2 Hz -> 0.75 Hz -> 1.35 Hz -> 0.9 Hz
```

这类分段突变步频会让当前单层 AO 来不及重新锁相，表现为：

- `phase_rmse_percent` 升高；
- `phase_abs_error_p95_percent` 升高；
- `frequency_rmse_hz` 升高；
- 频率变化导致力矩变化率接近或超过阈值。

这和文献 **Effective Prediction of Gait Phase for Assisted Walking by Means of Gait-Based Adaptive Oscillators**（IEEE T-ASE 2025, DOI: `10.1109/TASE.2024.3520148`）讨论的问题一致：传统 AO 在稳态行走中有效，但在频繁 stop/go 或步频变化时通常需要多个 stride 才能同步，可能导致过渡期相位预测不准。

### 文献思路摘要

该论文提出 gait-based adaptive oscillator，核心是 **two-level AO systems**：

1. **高层 AO**
   - 从上一 stride 学习步态参数；
   - 提取上一 stride 的频率、幅值、相位等高层参数；
   - 在新 stride 初期把学习到的参数传给低层 AO。

2. **低层 AO**
   - 实时估计当前 stride 的相位；
   - 不完全从零开始适应，而是使用高层 AO 提供的初始/更新参数；
   - 目标是在非稳态行走中更快同步。

论文目标是减少传统 AO 在步频变化时的多周期收敛时间，使外骨骼辅助能更快、更稳定地跟上 stop/go 和 cadence change。

### 当前 V2 的轻量化可行方案

不建议一开始完整复刻论文中的双层 AO。当前 V2 可先实现一个轻量版：

> **Anchor-based high-level frequency update**

利用现有 `PhaseEstimator` 中的峰/谷 anchor 检测，在检测到可靠 anchor 后，用相邻 anchor 的时间间隔估计步频，并快速融合到低层 AO 的 `omega`。

#### 初版算法草案

峰到谷是半个周期，因此：

```cpp
measured_frequency_hz = 0.5 / (current_anchor_time_s - last_anchor_time_s);
```

然后对低层 AO 的频率做快速融合：

```cpp
oscillator_.omega =
    (1.0 - anchor_frequency_gain) * oscillator_.omega +
    anchor_frequency_gain * measured_frequency_hz * 2.0 * M_PI;
```

建议配置：

```cpp
struct PhaseConfig {
    ...
    double anchor_frequency_gain = 0.45;
    double anchor_frequency_min_hz = 0.45;
    double anchor_frequency_max_hz = 1.60;
};
```

必须保留保护条件：

- anchor 间隔满足 `anchor_min_fraction_of_period`；
- `spread_deg > peak_min_spread_deg`；
- `measured_frequency_hz` 在合理范围内；
- 停步 `Stopping` / `Frozen` 期间不更新高层频率。

### 预期收益

优先改善：

- `multi_rate` 的 `frequency_rmse_hz`；
- `multi_rate` 的 `phase_rmse_percent` 和 `phase_abs_error_p95_percent`；
- 步频突变后 AO 的重新锁相时间：
  - `frequency_adaptation_time_mean_s`
  - `phase_adaptation_time_mean_s`
  - `combined_adaptation_time_mean_s`

不应破坏：

- `steady_0p8`；
- `freq_ramp`；
- `amp_ramp`；
- `stop_go`；
- `abrupt_stop`。

### 风险

- 噪声可能制造假 anchor，导致频率被错误快速更新；
- 频率融合增益过大可能让 AO 抖动；
- 对 `noisy_stop_go` 这类曲线，可能需要先增强峰/谷事件检测的抗噪性。

### TODO

- [ ] 在 `PhaseConfig` 中加入 anchor-based frequency update 参数：
  - `anchor_frequency_gain`
  - `anchor_frequency_min_hz`
  - `anchor_frequency_max_hz`
- [ ] 在 `PhaseEstimator` 中保存上一可靠 anchor 的时间和类型。
- [ ] 在检测到新可靠 anchor 时，用 anchor 间隔估算 stride frequency。
- [ ] 将估算频率融合进低层 AO 的 `oscillator_.omega`。
- [ ] 确保 `Stopping` / `Frozen` 期间不更新该高层频率。
- [ ] 增加 C++ 回归测试：频率突变时 `PhaseEstimator` 能更快更新频率。
- [ ] 重跑离线曲线：
  - `steady_0p8`
  - `freq_ramp`
  - `amp_ramp`
  - `stop_go`
  - `abrupt_stop`
  - `multi_rate`
  - `repeated_stop_go`
- [ ] 若 `noisy_stop_go` 变差，增加 anchor 过零滞回 / 最小显著性 / 事件确认机制。

### 当前结论

该方向适合当前 V2 的 `multi_rate` 问题。它不是单纯调参，而是给单层 AO 增加一个“高层 stride 参数学习/快速频率重估”机制，属于论文双层 AO 思路的轻量工程化版本。
