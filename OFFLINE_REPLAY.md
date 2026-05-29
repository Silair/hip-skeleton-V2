# V2 离线曲线回放

目的：在不上实机、不触碰 Kvaser/CAN/底层电机控制的情况下，把人工设计或录制的关节角曲线喂给 V2 控制层，生成与实机运行相同结构的 runtime CSV，再用 `tools/analyze_run.py` 计算相位、频率、力矩、安全门控等指标。

## 工作流

```bash
# 1) 生成一条可控输入曲线（也可以自己手写 CSV）
python3 hs_exoskeleton_v2/tools/make_replay_curve.py \
  --output /tmp/hsx_replay_curve.csv \
  --scenario sine \
  --duration-s 12 \
  --rate-hz 50 \
  --amplitude-rad 0.35 \
  --frequency-hz 0.8

# 2) 编译离线回放器（不链接 Kvaser/CANLIB）
g++ -std=c++17 \
  -I2026.4.14hs_exoskeleton/include \
  -Ihs_exoskeleton_v2 \
  hs_exoskeleton_v2/tools/replay_control.cpp \
  hs_exoskeleton_v2/control/GaitFeatureExtractor.cpp \
  hs_exoskeleton_v2/control/PhaseEstimator.cpp \
  hs_exoskeleton_v2/control/IntentDetector.cpp \
  hs_exoskeleton_v2/control/FreezeManager.cpp \
  hs_exoskeleton_v2/control/StopDetector.cpp \
  hs_exoskeleton_v2/control/AssistStateMachine.cpp \
  hs_exoskeleton_v2/control/TorqueProfile.cpp \
  hs_exoskeleton_v2/logging/ExoLogger.cpp \
  -o /tmp/hsx_v2_replay

# 也可以用 CMake 目标 hs_exoskeleton_v2_replay。

# 3) 回放，生成控制层 runtime CSV
/tmp/hsx_v2_replay \
  --input /tmp/hsx_replay_curve.csv \
  --output-dir /tmp/hsx_replay_logs \
  --prefix sine_replay

# 4) 分析输出 CSV，生成指标和 HTML 图表
python3 hs_exoskeleton_v2/tools/analyze_run.py \
  /tmp/hsx_replay_logs/<date>/sine_replay_1_<date>.csv
```

## 输入曲线 CSV 格式

必需列：

```csv
TimeS,LeftJointPosRad,RightJointPosRad
```

可选列：

```csv
LeftJointVelRadS,RightJointVelRadS,Healthy,Enabled
```

如果没有速度列，回放器会按时间戳做有限差分。`Healthy=0` 可用于离线验证故障/冻结/卸力路径。

## 设计自己的曲线

`make_replay_curve.py` 支持内置曲线：

- `sine`：稳定周期步态；适合看 AO 稳态相位误差、力矩峰值相位。
- `stop_go`：走-停-走；适合看停步、恢复、静止误助力。
- `freq_ramp`：步频线性变化；适合看频率追踪与收敛。
- `amplitude_ramp`：步幅从小到大再变小；适合看助力进入/退出边界。
- `abrupt_stop`：正常行走后停在非零开合姿态；适合复现“停下后大 spread 误助力”。
- `asymmetric`：左右幅值和相位略不一致；适合看真实非对称步态下的相位/力矩鲁棒性。
- `repeated_stop_go`：多次走停循环；适合压力测试停步/恢复、残余力矩和多次状态切换。
- `multi_rate`：分段多速率步态；适合压力测试 AO 对突变步频的重新锁相能力。
- `custom`：用表达式直接设计左右关节角。

示例：

```bash
python3 hs_exoskeleton_v2/tools/make_replay_curve.py \
  --output /tmp/custom_curve.csv \
  --scenario custom \
  --duration-s 10 \
  --rate-hz 50 \
  --left-expr  '0.12 + 0.18*sin(tau*0.9*t)' \
  --right-expr '0.08 + 0.16*sin(tau*0.9*t + 0.25)'
```

表达式可用变量/函数：`t`, `pi`, `tau`, `sin`, `cos`, `sqrt`, `exp`, `min`, `max` 等。

> 注意：当前 `GaitFeatureExtractor` 用 `left.position_rad + right.position_rad` 作为标量步态信号。默认生成器会把设计的标量信号平均分到左右两侧，以便在这个符号约定下产生清晰相位。若你的真实数据左右符号相反，可以在自定义表达式或 CSV 预处理时做符号转换。

## 建议批量测试曲线

最小上机前离线套件建议覆盖：

```bash
python3 tools/make_replay_curve.py --output /tmp/steady_0p8.csv --scenario sine --duration-s 12 --rate-hz 50 --amplitude-rad 0.35 --frequency-hz 0.8
python3 tools/make_replay_curve.py --output /tmp/freq_ramp.csv --scenario freq_ramp --duration-s 18 --rate-hz 50 --amplitude-rad 0.35 --frequency-hz 0.6 --ramp-end-frequency-hz 1.2
python3 tools/make_replay_curve.py --output /tmp/amp_ramp.csv --scenario amplitude_ramp --duration-s 18 --rate-hz 50 --amplitude-rad 0.40 --min-amplitude-rad 0.08 --frequency-hz 0.8
python3 tools/make_replay_curve.py --output /tmp/stop_go.csv --scenario stop_go --duration-s 18 --rate-hz 50 --amplitude-rad 0.35 --frequency-hz 0.8
python3 tools/make_replay_curve.py --output /tmp/abrupt_stop.csv --scenario abrupt_stop --duration-s 12 --rate-hz 50 --amplitude-rad 0.35 --frequency-hz 0.8 --stop-time-s 5.2
python3 tools/make_replay_curve.py --output /tmp/asymmetric.csv --scenario asymmetric --duration-s 12 --rate-hz 50 --amplitude-rad 0.35 --frequency-hz 0.8
python3 tools/make_replay_curve.py --output /tmp/repeated_stop_go.csv --scenario repeated_stop_go --duration-s 24 --rate-hz 50 --amplitude-rad 0.35 --frequency-hz 0.8 --stop-cycle-s 4.0 --stop-window-s 1.0
python3 tools/make_replay_curve.py --output /tmp/multi_rate.csv --scenario multi_rate --duration-s 20 --rate-hz 50 --amplitude-rad 0.35 --rate-sequence-hz 0.6,1.2,0.75,1.35,0.9
```

其中 `sine`、`freq_ramp`、`amplitude_ramp`、`stop_go`、`abrupt_stop` 应优先 PASS。`repeated_stop_go` 和 `multi_rate` 是更严格的压力测试：前者重点看多次停步恢复后的残余力矩和峰值相位边界，后者重点看步频突变时 AO 重新锁相和力矩变化率。

## 输出与评价

回放器输出的 CSV 表头与实机 V2 日志一致，后处理仍使用：

```bash
python3 hs_exoskeleton_v2/tools/analyze_run.py <runtime.csv>
```

重点看：

- `phase_rmse_percent` / `phase_mae_percent`：AO 相位追踪误差。
- `frequency_rmse_hz`：步频追踪误差。
- `convergence_time_s_at_5_percent`：进入 5% 相位误差所需时间。
- `peak_torque_phase_mae_deg`：助力峰值相位误差。
- `freeze_torque_violation_count`、`disallow_output_torque_violation_count`：冻结/禁止输出时是否仍有力矩。
- `stationary_false_assist_s`：停住时误助力时长。

建议先用 `sine`、`freq_ramp`、`stop_go` 三类曲线跑过，再考虑实机。
