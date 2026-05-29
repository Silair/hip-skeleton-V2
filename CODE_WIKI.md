# hs_exoskeleton_v2 代码百科 (CODE_WIKI)

> V2 是与旧版 `hs_exoskeleton.cpp` 并行的新外骨骼控制管线：把硬件访问、步态特征、相位估计、意图/冻结、助力状态机、力矩曲线、日志拆成独立模块，控制层只依赖关节空间抽象。

## 1. 项目概述 (Project Overview)

`hs_exoskeleton_v2` 是旧版外骨骼控制链路的模块化重构版本。`README.md` 明确目标是保留旧版控制循环以便对照/回滚，同时新增一条 V2 控制管线：控制层在关节空间工作，不直接暴露 CAN/Kvaser 细节；用启发式运动置信度替代旧版硬编码 GMM/冻结/力矩窗口耦合。

V2 的程序入口是 `app/main.cpp`，主编排器是 `control/ExoController.cpp`。构建目标目前定义在相邻旧项目的 `../2026.4.14hs_exoskeleton/CMakeLists.txt`，本目录自身没有独立 `CMakeLists.txt`。

## 2. 仓库地图 (Repository Map)

### 顶层

- `README.md`：项目目标、目录结构、构建目标说明。
- `CODE_WIKI.md`：本文档。
- `app/main.cpp`：V2 应用入口，负责组装配置、硬件、日志器、控制器并运行。

### `config/`

- `config/ControlConfig.h`：控制环与算法超参，嵌套 `IntentConfig`、`FreezeConfig`、`AssistConfig`、`TorqueConfig`、`PhaseConfig`。
- `config/HardwareConfig.h`：CAN 通道/波特率、电机 ID、关节缩放、力矩上限、上电零位配置。
- `config/LoggingConfig.h`：日志根目录、文件前缀、同步会话 ID、数据流 ID。

### `control/`

- `control/ExoController.h/.cpp`：V2 主控制器；固定频率循环里串联所有控制子模块。
- `control/GaitFeatureExtractor.h/.cpp`：从左右关节角提取滤波步态信号、开合幅度、相位速度。
- `control/PhaseEstimator.h/.cpp`：使用旧版 `MultiHarmonicAO` 估计相位、频率、幅值；V2.1 在可靠峰/谷 anchor 处做置信度加权的频率校正（不校正相位）。
- `control/IntentDetector.h/.cpp`：把 spread、velocity、amplitude、frequency 归一化加权为运动置信度，停止概率为其补。
- `control/FreezeManager.h/.cpp`：Live/Frozen/Recovery 三态滞回，输出冻结请求、相位跟踪门控和恢复状态。
- `control/AssistStateMachine.h/.cpp`：Transparent/Tracking/Ramp/Active/Frozen/Fault 助力门控状态机。
- `control/TorqueProfile.h/.cpp`：按相位和频率生成左右对称助力力矩。
- `control/GaitFeatures.h`：控制管道中的 `GaitFeatures`、`IntentEstimate`、`PhaseEstimate` 轻量结构体。

### `hardware/`

- `hardware/IExoHardware.h`：硬件抽象接口，控制器只依赖此接口。
- `hardware/JointTypes.h`：关节空间状态、命令和健康枚举。
- `hardware/KvaserExoHardware.h/.cpp`：基于旧版 `selfDevManuipulator`/Kvaser 封装的硬件实现。
- `hardware/LegacyJointSpaceMapper.h/.cpp`：电机空间与关节空间之间的线性缩放、力矩限幅。

### `logging/`

- `logging/Clock.h/.cpp`：单调时间、近似 epoch 毫秒、相邻采样 dt。
- `logging/ExoLogger.h/.cpp`：按日期/序号创建 CSV，写入控制中间量。

### `tests/`

- `tests/test_control.cpp`：纯控制逻辑回归测试，覆盖意图、力矩、助力状态机、冻结滞回、特征提取、相位估计。
- `tests/test_clock.cpp`：`Clock` 时间单调性和 dt 测试。

## 3. 技术栈与运行方式 (Stack and Commands)

- 语言：C++17。
- 构建入口：相邻旧项目 `../2026.4.14hs_exoskeleton/CMakeLists.txt`。
- 硬件依赖：Kvaser CANLIB、旧版 `include/Kvaser_H/*`、`include/Robot_H/selfDevManuipulator.h`、Eigen。
- 运行时可选环境变量：`HSX_SYNC_SESSION_ID`、`HSX_STREAM_ID`，在 `app/main.cpp` 中写入 `LoggingConfig`。

在相邻旧项目目录下构建：

```bash
cd ../2026.4.14hs_exoskeleton
cmake -S . -B build \
  -DKVASER_SDK_INCLUDE_DIR=<Kvaser include dir> \
  -DKVASER_SDK_LIB_DIR=<Kvaser lib dir> \
  -DEIGEN3_INCLUDE_DIR=<Eigen include dir>
cmake --build build --target hs_exoskeleton_v2
cmake --build build --target hs_exoskeleton_v2_tests
cmake --build build --target hs_exoskeleton_v2_clock_tests
ctest --test-dir build -R hs_exoskeleton_v2
```

待确认：本目录缺少独立构建文件；如果要独立维护 V2，应新增 V2 自身的 `CMakeLists.txt` 或把旧项目中的 V2 target 迁移过来。

## 4. 整体架构 (Overall Architecture)

V2 的主数据流：

```text
app/main.cpp
  └─ ExoController
      ├─ IExoHardware / KvaserExoHardware
      │   └─ LegacyJointSpaceMapper
      ├─ GaitFeatureExtractor
      ├─ PhaseEstimator (MultiHarmonicAO)
      ├─ IntentDetector
      ├─ FreezeManager
      ├─ AssistStateMachine
      ├─ TorqueProfile
      └─ ExoLogger / Clock
```

核心设计边界：

- `control/` 只面向 `ExoState`、`ExoCommand`、算法结构体，不直接操作 CAN。
- `hardware/IExoHardware.h` 是控制层与硬件层的边界，可用于后续测试桩、仿真或非 Kvaser 实现。
- `hardware/KvaserExoHardware.cpp` 仍依赖旧版硬件封装和 Kvaser API，负责实际总线初始化、读寄存器、下发力矩、急停和下电。
- `logging/ExoLogger.cpp` 只记录控制中间量，不影响控制决策。

## 5. 核心模块详解 (Core Modules)

### 5.1 应用入口：`app/main.cpp`

职责：创建三套默认配置 `HardwareConfig`、`ControlConfig`、`LoggingConfig`；读取 `HSX_SYNC_SESSION_ID`、`HSX_STREAM_ID` 覆盖日志标识；构造 `KvaserExoHardware`、`ExoLogger`、`ExoController`；执行 `initialize()`、`run()`、`shutdown()`。

### 5.2 主控制器：`control/ExoController.h/.cpp`

职责：V2 控制环编排，不实现具体算法细节。主要顺序：

1. `initialize()`：硬件初始化、使能、零位标定、打开日志。
2. `run()`：按 `ControlConfig::loop_frequency_hz` 运行到 `run_duration_s`。
3. 每圈读 `ExoState`，提特征，估相位，估意图，更新冻结，更新助力状态，计算力矩，写日志，下发命令。
4. 硬件读/写失败时调用 `hardware_.emergencyStop()` 并返回失败。
5. `shutdown()`：先关日志再关硬件。

重要门控：`command.allow_output = assist.allow_output && !freeze.freeze_requested && state.healthy`，冻结或硬件不健康时禁止有效力矩输出。

### 5.3 配置：`config/*.h`

- `ControlConfig.h`：默认控制频率 50 Hz，运行 180 s；子配置覆盖意图阈值、冻结滞回、warmup 锚点数、力矩增益/限幅、AO 频率范围/低通/锚点条件。
- `HardwareConfig.h`：默认 CAN channel 0、1 Mbps、左右电机 ID `0x0001/0x0002`；左右位置/速度缩放与旧版 `SCALE_L/SCALE_R` 对齐；最大关节力矩 8 N·m。
- `LoggingConfig.h`：默认写到 `Data/<date>/AO_v2_test_<n>_<date>.csv`，并记录 sync/session/stream 标识。

### 5.4 步态特征：`control/GaitFeatureExtractor.h/.cpp`

输入：`hardware/JointTypes.h` 中的 `ExoState`。

处理：左右关节角相加得到 `phase_signal_rad`，一阶低通得到 `filtered_phase_signal_rad`，绝对值转度数为 `spread_deg`，相邻滤波信号差分得到 `phase_velocity_deg_s`。

输出：`control/GaitFeatures.h` 中的 `GaitFeatures`。

### 5.5 相位估计：`control/PhaseEstimator.h/.cpp`

输入：`GaitFeatures`、dt、`tracking_enabled`、**上一帧** `AssistState`（因相位更新在助力状态机之前执行）。

处理：

- 复用旧版 `MultiHarmonicAO`，构造时初始化为 `PhaseConfig::ao_initial_frequency_hz`。
- tracking 开启时把本周期输入分小步喂给 AO，并将频率夹在 `ao_min/max_frequency_hz` 范围内。
- 用 `signed_phase_velocity_deg_s` 的符号换向 + `anchor_min_velocity_deg_s` + `spread_deg` 检测可靠 Peak/Valley anchor。
- 异类相邻 anchor 估计半周期频率，经 confidence、绝对/相对/step 限幅后，按 `effective_gain = anchor_frequency_gain(_ramp) * confidence` 融合到目标频率，并以 `max_omega_rate_rad_s2` 限速写回 `oscillator_.omega`。
- `AssistState::Active`/`Ramp` 允许频率更新；`Tracking` 只累计 anchor；`Stopping`/`Frozen`/`Transparent`/`Fault` 禁止；`tracking_enabled` 关闭时重置 anchor 状态机。
- V2.1 不做锚点相位校正（`anchor_phase_gain = 0`）；输出相位为 `wrapAngle(phi_GP)`。

输出：`PhaseEstimate`，含 `phase_rad`、`frequency_hz`、`amplitude_rad`、`valid`、`anchor_detected`，以及 `anchor_frequency_updated`、`anchor_rejected`、`anchor_reject_reason`（V2.2）、`anchor_confidence`、`omega_correction_hz` 等调试字段。

V2.2：`anchor_confirm_delay_frames` 延迟 1 帧确认峰/谷；`AnchorRejectReason` 与 `AnchorCandidate` 日志；Tracking 可靠 anchor 可 `enable_tracking_deferred_frequency` 延迟到 Ramp/Active 再校正频率。

V2.2.1：`StartupPrior` 在 Tracking/Ramp 检测起步趋势，混合 `omega_target`（Tracking 半增益、Ramp/Active 全增益）；见 `docs/V22_1_STARTUP_PRIOR.md`。

### 5.6 意图检测：`control/IntentDetector.h/.cpp`

输入：`GaitFeatures`，其中 `ExoController` 会用 `PhaseEstimate` 回填 `amplitude_rad`、`frequency_hz`。

处理：把 spread、phase velocity、amplitude、frequency 分别按配置阈值线性归一化到 0~1，按 0.35/0.30/0.20/0.15 加权得到 raw motion confidence，并用 `smoothing_alpha` 一阶平滑。

输出：`IntentEstimate`，其中 `stop_probability = 1 - motion_confidence`。

### 5.7 冻结管理：`control/FreezeManager.h/.cpp`

状态：`Live`、`Frozen`、`Recovery`。

规则：

- Live：停止概率持续超过 `enter_stop_probability` 和 `enter_hold_seconds` 后进入 Frozen。
- Frozen：停止概率低于 `exit_stop_probability` 且运动置信度超过 `resume_motion_confidence` 持续满足后进入 Recovery。
- Recovery：运动置信度持续满足后回 Live，否则退回 Frozen。

输出 `FreezeDecision`：是否请求冻结、是否允许相位跟踪、是否处于恢复窗口。

### 5.8 助力状态机：`control/AssistStateMachine.h/.cpp`

状态：`Transparent`、`Tracking`、`Ramp`、`Active`、`Frozen`、`Fault`。

- Transparent：无力矩，运动置信度和有效相位满足后进入 Tracking。
- Tracking：累计相位锚点，达到 `warmup_anchor_count` 后进入 Ramp。
- Ramp：按 `ramp_up_rate_per_s` 增加 `torque_scale`，接近 1 后 Active。
- Active：保持 `torque_scale = 1`。
- Frozen：冻结请求期间按 `ramp_down_rate_per_s` 降低力矩比例；解除后回 Tracking。
- Fault：故障锁定，输出 0。

输出 `AssistOutput` 决定 `TorqueProfile` 是否允许产生非零力矩。

### 5.9 力矩曲线：`control/TorqueProfile.h/.cpp`

输入：相位、频率、`torque_scale`、`allow_output`。

处理：按频率在 `base_gain_min_nm` 到 `base_gain_max_nm` 之间插值，乘以 `torque_scale`；相位加 `lead_angle_rad` 后，用 `cos` 正半轴给右腿、负半轴给左腿；最后按 `max_torque_nm` 夹紧。

输出：`TorqueCommand { left_nm, right_nm }`。

### 5.10 硬件层：`hardware/*.h/.cpp`

- `IExoHardware`：定义 `initialize()`、`enable()`、`calibrateZero()`、`readState()`、`applyCommand()`、`emergencyStop()`、`shutdown()`。
- `JointTypes`：`ExoState` 包含时间、loop seq、左右 `JointState`、enabled/healthy/error；`ExoCommand` 包含左右关节力矩和 `allow_output`。
- `KvaserExoHardware`：调用旧版 `selfDevManuipulator<2>`；初始化 CAN，左右电机使能和零位；读位置/速度寄存器并转换到关节空间；`allow_output=false` 时强制下发 0 力矩；shutdown 时 brake 和电机下电。
- `LegacyJointSpaceMapper`：按 `HardwareConfig` 的缩放系数把电机位置/速度映射到关节位置/速度，并把关节力矩转换为电机命令且夹紧。

### 5.11 日志与时间：`logging/*.h/.cpp`

- `Clock`：用 steady clock 作为单调时间基准，用构造时 system clock 锚点近似生成 epoch ms；`dtSeconds()` 在 `.cpp` 中实现并被控制器/测试使用。
- `ExoLogger`：`open()` 创建日期目录并写表头；`write()` 输出 sync/session、loop、时间、健康、助力/冻结状态、意图、相位、特征、关节状态、力矩命令；析构和 `close()` 关闭文件。

待确认：`logging/Clock.h` 当前注释提到 `dtSeconds()`，但头文件可见区域未声明该 public 方法，而 `logging/Clock.cpp`、`control/ExoController.cpp`、`tests/test_clock.cpp` 都调用/定义它；实际编译可能暴露头文件声明缺失问题，需构建确认。

## 6. 关键执行流程 (Core Flows)

### 6.1 初始化

1. `app/main.cpp` 构造配置。
2. 环境变量覆盖日志 session/stream ID。
3. 构造硬件、日志器、控制器。
4. `ExoController::initialize()`：`hardware.initialize()` → `hardware.enable()` → `hardware.calibrateZero()` → `logger.open()`。

### 6.2 每圈控制

1. `Clock` 计算 measured dt，控制用 dt 被夹在 `[0.0001, 0.1]`。
2. `IExoHardware::readState()` 填充 `ExoState`；失败则急停返回。
3. `GaitFeatureExtractor::update()` 生成步态特征。
4. `PhaseEstimator::update()` 输出相位/频率/幅值；冻结时可关闭 tracking。
5. `ExoController` 把相位估计的幅值/频率写回特征。
6. `IntentDetector::update()` 输出运动置信度和停止概率。
7. `FreezeManager::update()` 输出冻结决策。
8. `AssistStateMachine::update()` 输出状态、力矩比例和 allow_output。
9. `TorqueProfile::compute()` 生成左右力矩。
10. `ExoCommand` 组合最终门控，先写 CSV，再 `hardware.applyCommand()`。
11. `sleep_until(next_tick)` 对齐下一周期。

### 6.3 故障路径

- 初始读状态失败：`run()` 返回 false。
- 环内读状态或下发命令失败：调用 `hardware_.emergencyStop()`，然后返回 false。
- `state.healthy=false`：助力状态机收到 `faulted=true`，进入 Fault；最终 `command.allow_output=false`。

### 6.4 关闭

`ExoController::shutdown()` 先 `logger.close()`，再 `hardware.shutdown()`；Kvaser 实现中 shutdown 会 brake，并在已初始化时左右电机下电。

## 7. 配置、状态与持久化 (Config, State, Persistence)

- 配置全部是 C++ 头文件默认值，目前没有运行时配置文件。
- 日志目录由 `LoggingConfig::base_folder` 决定，默认 `Data/<YYYY.MM.DD>/`。
- `ExoLogger` 表头是 CSV 后处理的契约；改字段顺序时必须同步 `write()`。
- `ControlConfig` 是算法调参入口；硬件缩放/限幅必须在 `HardwareConfig` 调整，避免控制算法直接关心电机空间。

## 8. 扩展点与安全边界 (Extension Points and Safety)

- 增加仿真/测试硬件：实现 `IExoHardware`，无需改 `ExoController`。
- 调整助力手感：优先改 `TorqueConfig` 与 `AssistConfig`。
- 调整冻结策略：改 `FreezeConfig` 或 `FreezeManager`，保留滞回/持续时间，避免抖动。
- 调整步态判断：改 `IntentConfig` 权重或阈值，验证 `tests/test_control.cpp`。
- 安全边界：`allow_output`、`max_joint_torque_nm`、`max_torque_nm`、故障急停、shutdown 下电逻辑不能轻易绕过。
- 不建议让 `control/` 直接包含 Kvaser API；保持 `hardware/IExoHardware.h` 边界。

## 9. 测试与验证 (Tests and Verification)

### 已有测试

`tests/test_control.cpp` 覆盖：

- 运动时意图检测倾向 motion。
- 静止时意图检测倾向 stop。
- 力矩曲线左右相位对称。
- 助力状态机 warmup 后进入 active/ramp。
- 冻结管理使用滞回。
- 特征提取能生成 spread 与 velocity。
- 相位估计能跟踪周期信号并输出有效频率。

`tests/test_clock.cpp` 覆盖：

- epoch ms 非递减。
- elapsed seconds 增长。
- dt 非负。

### 推荐验证命令

在 `../2026.4.14hs_exoskeleton` 下：

```bash
cmake --build build --target hs_exoskeleton_v2_tests
cmake --build build --target hs_exoskeleton_v2_clock_tests
ctest --test-dir build -R hs_exoskeleton_v2
```

本次只生成文档，未执行构建；原因是当前环境缺少已确认的 Kvaser SDK/Eigen/CANLIB Windows 构建上下文。后续修改 V2 源码时应先跑纯控制测试，再上硬件。

## 10. 开发导航与维护建议 (Developer Navigation)

- 看总体流程：`app/main.cpp` → `control/ExoController.cpp::initialize/run/shutdown`。
- 调参：先看 `config/ControlConfig.h`、`config/HardwareConfig.h`、`config/LoggingConfig.h`。
- 看算法：`control/GaitFeatureExtractor.cpp`、`control/PhaseEstimator.cpp`、`control/IntentDetector.cpp`、`control/FreezeManager.cpp`、`control/AssistStateMachine.cpp`、`control/TorqueProfile.cpp`。
- 看硬件边界：`hardware/IExoHardware.h`、`hardware/KvaserExoHardware.cpp`。
- 看日志字段：`logging/ExoLogger.cpp` 的表头和 `write()`。
- 改动时优先补/更新 `tests/test_control.cpp`；硬件相关改动至少需要通过假硬件或实机低风险流程验证。

## 更新记录 (Changelog)

- 2026-05-28：创建 CODE_WIKI；基于 `README.md`、`app/main.cpp`、`config/`、`control/`、`hardware/`、`logging/`、`tests/` 和相邻 CMake 目标整理代码结构与控制流程。

## 离线回放工具

- `tools/make_replay_curve.py`：生成可回放的关节角 CSV，支持 `sine`、`stop_go`、`freq_ramp` 和表达式 `custom` 曲线。
- `tools/replay_control.cpp`：不接硬件，直接把 CSV 样本喂给 `GaitFeatureExtractor -> PhaseEstimator -> IntentDetector -> FreezeManager -> AssistStateMachine -> TorqueProfile -> ExoLogger`，输出与实机日志同结构的 runtime CSV。
- `OFFLINE_REPLAY.md`：说明曲线格式、编译运行命令，以及如何接 `tools/analyze_run.py` 评估控制效果。

