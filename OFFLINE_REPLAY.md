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

## MuJoCo 单腿 dry-run

如果已经有单腿 MuJoCo 模型，可以先把仿真关节角导出为同一套 replay CSV，再复用上面的 V2 离线控制器与 `analyze_run.py`。这一步只验证“MuJoCo 状态 -> V2 控制算法 -> 日志评价”，不把 V2 力矩回写到 MuJoCo 模型。

当前本机已验证的模型路径：

```bash
/home/lin/exo_mujoco_sim/models/hip_exo_1dof.xml
```

生成单腿 dry-run 曲线：

```bash
/home/lin/miniconda3/envs/mujoco/bin/python tools/mujoco_dry_run_curve.py \
  --mjcf /home/lin/exo_mujoco_sim/models/hip_exo_1dof.xml \
  --output /tmp/hsx_mujoco_dry_run/hip_1dof_curve.csv \
  --joint hip \
  --driver-actuator hip_human_motor \
  --duration-s 12 \
  --sample-rate-hz 50 \
  --amplitude-deg 40 \
  --frequency-hz 0.8 \
  --driver-kp 80 \
  --driver-kd 8 \
  --virtual-right copy
```

说明：

- `--driver-actuator` 只作为仿真里的“人体驱动”让单腿摆动；V2 计算出的外骨骼力矩不会施加到 MuJoCo。
- `--virtual-right copy` 会把单腿角度复制为右侧虚拟输入，使当前 `left + right` 步态信号有足够幅值触发 anchor。若想更保守地只用单腿输入，可改为 `--virtual-right zero`，但可能无法进入助力状态。
- 输出 CSV 可直接喂给 `hs_exoskeleton_v2_replay`，再用 `tools/analyze_run.py` 生成报告。

### MuJoCo 实时 dry-run

如果需要确认 V2 控制链能在 MuJoCo step 循环内直接运行，可编译 `tools/mujoco_realtime_dry_run.cpp`。它每 50 Hz 从 MuJoCo 读髋关节状态、运行 V2 控制模块、写标准 runtime CSV；V2 算出的力矩只记录，不施加回 MuJoCo，`hip_exo_motor` 会被强制保持 0。

```bash
g++ -std=c++17 \
  -I. \
  -I/home/lin/code/2026.4.14hs_exoskeleton/include \
  -I/home/lin/miniconda3/envs/mujoco/lib/python3.11/site-packages/mujoco/include \
  tools/mujoco_realtime_dry_run.cpp \
  control/GaitFeatureExtractor.cpp control/PhaseEstimator.cpp \
  control/IntentDetector.cpp control/FreezeManager.cpp \
  control/AssistStateMachine.cpp control/TorqueProfile.cpp \
  control/StopDetector.cpp control/StopTorqueLimiter.cpp \
  logging/ExoLogger.cpp logging/Clock.cpp \
  /home/lin/miniconda3/envs/mujoco/lib/python3.11/site-packages/mujoco/libmujoco.so.3.9.0 \
  -Wl,-rpath,/home/lin/miniconda3/envs/mujoco/lib/python3.11/site-packages/mujoco \
  -o /tmp/hsx_mujoco_realtime_dry_run

/tmp/hsx_mujoco_realtime_dry_run \
  --mjcf /home/lin/exo_mujoco_sim/models/hip_exo_1dof.xml \
  --output-dir /tmp/hsx_mujoco_realtime_out/logs \
  --prefix realtime_hip_1dof \
  --joint hip \
  --driver-actuator hip_human_motor \
  --exo-actuator hip_exo_motor \
  --duration-s 12 \
  --sample-rate-hz 50 \
  --amplitude-deg 40 \
  --frequency-hz 0.8 \
  --driver-kp 80 \
  --driver-kd 8 \
  --virtual-right copy

python3 tools/analyze_run.py \
  /tmp/hsx_mujoco_realtime_out/logs/<date>/realtime_hip_1dof_1_<date>.csv \
  --output-dir /tmp/hsx_mujoco_realtime_out/report
```

小增益闭环可以在确认 dry-run 正常后开启：

```bash
/tmp/hsx_mujoco_realtime_dry_run \
  --mjcf /home/lin/exo_mujoco_sim/models/hip_exo_1dof.xml \
  --output-dir /tmp/hsx_mujoco_closed_loop_005/logs \
  --prefix closed_loop_005_hip_1dof \
  --joint hip \
  --driver-actuator hip_human_motor \
  --exo-actuator hip_exo_motor \
  --duration-s 12 \
  --sample-rate-hz 50 \
  --amplitude-deg 40 \
  --frequency-hz 0.8 \
  --driver-kp 80 \
  --driver-kd 8 \
  --virtual-right copy \
  --apply-v2-torque-scale 0.05
```

`--apply-v2-torque-scale 0.05` 表示将 V2 的左侧力矩乘以 5% 后施加到单腿模型的 `hip_exo_motor`，并按 MuJoCo actuator `ctrlrange` 限幅。runtime CSV 中仍记录 V2 原始计算力矩；实际施加到 MuJoCo 的力矩为该列乘以比例。

停走/急停安全边界可通过驱动场景参数扫描：

```bash
# 走 4s，停 3s，再恢复行走
/tmp/hsx_mujoco_realtime_dry_run \
  --mjcf /home/lin/exo_mujoco_sim/models/hip_exo_1dof.xml \
  --output-dir /tmp/hsx_mujoco_safety_sweep/stop_go/scale_100/logs \
  --prefix stop_go_100_hip_1dof \
  --duration-s 14 \
  --driver-scenario stop_go \
  --stop-start-s 4 \
  --stop-duration-s 3 \
  --apply-v2-torque-scale 1.0

# 走到 4s 后急停并保持停止姿态
/tmp/hsx_mujoco_realtime_dry_run \
  --mjcf /home/lin/exo_mujoco_sim/models/hip_exo_1dof.xml \
  --output-dir /tmp/hsx_mujoco_safety_sweep/abrupt_stop/scale_100/logs \
  --prefix abrupt_stop_100_hip_1dof \
  --duration-s 12 \
  --driver-scenario abrupt_stop \
  --stop-start-s 4 \
  --apply-v2-torque-scale 1.0
```

`--driver-scenario` 支持 `sine`、`stop_go`、`abrupt_stop`。安全边界优先看 `stationary_false_assist_s`、`anchor_update_during_stop_count`、`freeze_torque_violation_count`、`disallow_output_torque_violation_count`。

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
- `frequency_transition_count`：曲线里检测到的步频阶跃次数。
- `frequency_adaptation_time_mean_s` / `phase_adaptation_time_mean_s` / `combined_adaptation_time_mean_s`：步频突变后频率、相位、二者同时重新达标的平均耗时；适合比较 `multi_rate` 改进前后。
- `peak_torque_phase_mae_deg`：助力峰值相位误差。
- `freeze_torque_violation_count`、`disallow_output_torque_violation_count`：冻结/禁止输出时是否仍有力矩。
- `stationary_false_assist_s`：停住时误助力时长。

改进前/改进后可用：

```bash
python3 tools/compare_metrics.py /tmp/before/metrics.json /tmp/after/metrics.json
```

建议先用 `sine`、`freq_ramp`、`stop_go` 三类曲线跑过，再考虑实机。
