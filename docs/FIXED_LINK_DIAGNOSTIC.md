# 固定连杆低力矩诊断实验

目的不是验证助力效果，而是判断连杆固定时电机侧编码器、速度尖峰、撤力停机、归零差异是否正常。

## 安全默认值

- 第一次只用 `0.5 Nm`，确认安全后再单独做 `1.0 Nm`、`2.0 Nm`。
- 固定连杆必须使用夹具、绑带、木板或限位架。
- 人不要站在连杆运动平面内，保留物理急停。
- 不要用手硬抓正在输出的连杆。

## 推荐运行

最简单的 0.5 Nm / 60 s 固定连杆测试：

```powershell
tools\run_fixed_link_0p5nm_60s.ps1 -Side left
```

左侧固定连杆、正确归零、0.5 Nm：

```powershell
tools\run_fixed_link_diagnostic.ps1 -Trial left -MaxTorqueNm 0.5 -DurationS 20
```

右侧固定连杆：

```powershell
tools\run_fixed_link_diagnostic.ps1 -Trial right -MaxTorqueNm 0.5 -DurationS 20
```

不归零对比：

```powershell
tools\run_fixed_link_diagnostic.ps1 -Trial zero_compare -MaxTorqueNm 0.5 -DurationS 20 -CalibrateOnStart $false
```

脚本会调用 `tools\run_hardware_trial.ps1`，并固定这些低力矩参数：

```text
HSX_MAX_TORQUE_NM = 0.5
HSX_BASE_GAIN_MIN_NM = 0.2
HSX_BASE_GAIN_MAX_NM = 0.5
HSX_RAMP_UP_RATE_PER_S = 0.25
HSX_RAMP_DOWN_RATE_PER_S = 2.0
```

## 离线判定

单个 CSV：

```bash
python3 tools/fixed_link_diagnostics.py Data/hardware_trials/<round>/<date>/<log>.csv --side left
```

归零 vs 不归零：

```bash
python3 tools/fixed_link_diagnostics.py <any_runtime.csv> \
  --side left \
  --zeroed-csv <zeroed.csv> \
  --unzeroed-csv <unzeroed.csv> \
  --output fixed_link_diagnostics.json
```

关键判据：

- `raw_motor_pos_delta_rad < 0.01`：比较正常。
- `0.01..0.03`：轻微柔顺或间隙。
- `> 0.03`：可疑。
- `> 0.05`：明显空程、柔顺或滑移。
- `max_abs_raw_motor_vel_rad_s > 0.1`：速度尖峰可疑。
- `active_or_ramp_after_stop_intent_count > 0`：停步意图出现后仍在 Ramp/Active。
- `moving_after_output_disabled_count > 0`：撤力后电机侧仍在动，更偏机械被动运动或回差。

已有 CSV 字段已经覆盖实验需要的原始量：

```text
LeftRawMotorPos, RightRawMotorPos
LeftRawMotorVel, RightRawMotorVel
LeftJointPos, RightJointPos
LeftJointVel, RightJointVel
AllowOutput
LeftTorqueCmd, RightTorqueCmd
PhaseSignalRad, FilteredPhaseSignalRad, SpreadDeg
MotionConfidence, StopProbability, AssistState
```
