# hs_exoskeleton_v2 简化评审记录

> 目的：记录 V2 相对 `2026.4.14hs_exoskeleton/hs_exoskeleton.cpp` 做了哪些简化，判断这些简化是否合理，检查是否误删/遗漏必要文件，并列出后续仍可简化的空间。原则：尽量保持底层电机控制、Kvaser CAN 通信、寄存器读写和电机使能/下电逻辑不修改。

## 结论摘要

| 结论 | 判断 | 依据 |
| --- | --- | --- |
| V2 的主要简化方向是合理的 | 高置信 | V1 把硬件、相位、意图、冻结、助力、力矩、日志集中在 `hs_exoskeleton.cpp`；V2 拆成 `config/`、`control/`、`hardware/`、`logging/`、`tests/`，主循环由 `control/ExoController.cpp` 编排。 |
| V2 没有直接修改底层电机控制 | 高置信 | V2 硬件实现仍通过旧的 `selfDevManuipulator`/`Kvaser` 封装进行 CAN 初始化、读寄存器、力矩模式、制动和下电。 |
| V2 纯控制层已摆脱 Eigen/矩阵依赖 | 高置信 | `hs_exoskeleton_v2/control/` 和 `tests/test_control.cpp` 不包含 Eigen；本地 `g++` 编译运行 `test_control` 成功。 |
| V2 硬件可执行目标仍会间接依赖 Eigen | 高置信 | `hardware/KvaserExoHardware.h` 持有 `selfDevManuipulator<2>`，该旧类包含 `<Eigen/Core>`、`TEXTIO.h`、`Timer.h` 等。 |
| `Clock::dtSeconds()` 接口遗漏已修复 | 高置信 | 已在 `Clock.h` 声明 `dtSeconds()`；本地重新编译运行 clock test 通过。 |
| V2 有符号速度峰/谷语义已恢复到代码层 | 中高置信 | `GaitFeatureExtractor` 现在同时输出绝对速度和 `signed_phase_velocity_deg_s`；`PhaseEstimator` 用有符号速度判断锚点。仍需实机/日志验证行为是否与 V1 足够一致。 |

## 1. V1 中被简化掉的内容

### 1.1 单文件主循环拆分为模块

**V1 状态：**

`2026.4.14hs_exoskeleton/hs_exoskeleton.cpp` 同时负责：

- include 硬件、AO、GMM、日志、Windows 线程优先级、计时器等依赖（`hs_exoskeleton.cpp:15-25`）。
- 硬编码控制参数、左右缩放、日志路径（`hs_exoskeleton.cpp:36-56`）。
- 定义低通滤波器和日志路径生成函数（`hs_exoskeleton.cpp:61-111`）。
- 上电、零位、主循环、冻结、相位补偿、力矩输出和下电（`hs_exoskeleton.cpp:114-587`）。

**V2 简化：**

- `app/main.cpp` 只做配置、硬件、日志、控制器组装与返回码处理（`app/main.cpp:14-43`）。
- `control/ExoController.cpp` 只做控制链编排，具体算法分发到子模块（`control/ExoController.cpp:20-29`、`48-123`）。
- 参数集中到 `config/ControlConfig.h` 和 `config/HardwareConfig.h`（`config/ControlConfig.h:7-70`、`config/HardwareConfig.h:8-31`）。

**判断：合理。** 这降低了主循环的认知负担，使每个模块可单独测试，且没有触碰 Kvaser 底层实现。

### 1.2 GMM 停止意图简化为启发式运动置信度

**V1 状态：**

V1 用 `IntentGMM` 对 `[q_diff_deg, diff_vel, amp_est, freq_hz]` 输出 `p_stop_raw`，再一阶平滑为 `p_stop_belief`（`hs_exoskeleton.cpp:245-253`）。

**V2 简化：**

`IntentDetector` 将 spread、velocity、amplitude、frequency 线性归一化并加权，输出 `motion_confidence`，`stop_probability = 1 - motion_confidence`（`control/IntentDetector.cpp:27-58`）。阈值集中在 `IntentConfig`（`config/ControlConfig.h:7-18`）。

**判断：基本合理，但需要实验校准。**

- 合理点：去掉硬编码 GMM 参数后，可读性和可调性更强，测试也覆盖了“明显运动/明显静止”两端（`tests/test_control.cpp:18-50`）。
- 风险：GMM 的概率分布边界被简化为线性阈值，可能改变临界步态、慢走、停步过渡时的误判率。建议保留 V1/V2 同步日志对比一段实验数据后再确认阈值。

### 1.3 冻结逻辑从散落 if 改为状态机

**V1 状态：**

冻结进入/退出、AO 状态重置、warm-up 重置混在主循环中（`hs_exoskeleton.cpp:255-293`）。

**V2 简化：**

`FreezeManager` 提供 `Live -> Frozen -> Recovery -> Live` 三态滞回，并输出 `freeze_requested`、`phase_tracking_enabled`、`recovery_active`（`control/FreezeManager.cpp:10-55`）。阈值集中到 `FreezeConfig`（`config/ControlConfig.h:20-27`）。

**判断：合理。** 冻结逻辑的持续时间和恢复条件变得明确，且 `ExoController` 用冻结结果门控相位和最终输出（`control/ExoController.cpp:85`、`90-110`）。

### 1.4 助力状态机从三态变为显式 warm-up/ramp/fault 状态

**V1 状态：**

V1 只有 `ASSIST_TRANSPARENT / ASSIST_ACTIVE / ASSIST_RELEASE` 三态（`hs_exoskeleton.cpp:58-59`），warm-up、fade-in、low_amp_counter、冻结退出都散落在主循环（`hs_exoskeleton.cpp:389-537`）。

**V2 简化：**

`AssistStateMachine` 显式建模 `Transparent / Tracking / Ramp / Active / Frozen / Fault`（`control/AssistStateMachine.cpp:10-75`）。warm-up 锚点数量、进入/退出置信度、爬升/下降速率在 `AssistConfig` 中配置（`config/ControlConfig.h:29-36`）。

**判断：合理。** 这不是“少状态”，而是“少隐式状态”。可读性和测试性提升明显，`tests/test_control.cpp` 已覆盖 warm-up 后进入 Ramp/Active（`tests/test_control.cpp:66-90`）。

### 1.5 力矩曲线从窗口式 if/else 简化为对称 cos 分配

**V1 状态：**

V1 力矩计算包含大量注释掉的候选策略，当前实际路径使用相位窗口、lead angle、`ASSIST_GAIN` 和 fade-in 输出左右力矩（`hs_exoskeleton.cpp:412-512`）。其中 `dynamic_assist_gain` 被计算但实际只在注释代码中使用（`hs_exoskeleton.cpp:420-429`、`431-475`、`479-509`）。

**V2 简化：**

`TorqueProfile` 用频率映射 gain，用 `max(0, -cos(phase+lead))` 给左腿，`max(0, cos(phase+lead))` 给右腿，并统一限幅（`control/TorqueProfile.cpp:22-41`）。相关参数在 `TorqueConfig`（`config/ControlConfig.h:38-46`）。

**判断：方向合理，但行为不等价。**

- 合理点：去掉注释残留和重复窗口分支，左右对称性有测试保护（`tests/test_control.cpp:52-64`）。
- 风险：V1 当前实际使用 `lead_angle = 0.35`，V2 默认 `lead_angle_rad = 0.20`；V1 的输出窗口与 V2 的连续半波 cos 分配不同。若目标是“行为尽量一致”，需要用旧日志对比力矩相位窗口。

### 1.6 日志从 ad hoc CSV 简化为结构化日志器

**V1 状态：**

V1 在主文件中生成日志路径，使用 `CsvLogger::writeCustomLog(...)` 写一长串位置参数（`hs_exoskeleton.cpp:83-111`、`547-553`）。

**V2 简化：**

`ExoLogger` 从 `ExoState/GaitFeatures/PhaseEstimate/IntentEstimate/FreezeDecision/AssistOutput/TorqueCommand` 写 CSV，表头和字段顺序在一个模块内维护（`logging/ExoLogger.cpp`，见 `CODE_WIKI.md` 说明）。

**判断：合理。** 写日志不再污染主控制循环，也更适合后续扩展同步 ID 和数据流 ID（`app/main.cpp:20-26`）。

### 1.7 计时器从 Windows/自定义高精度计时器简化为 `Clock` + `sleep_until`

**V1 状态：**

V1 包含 `Windows.h`，提升线程优先级（`hs_exoskeleton.cpp:22`、`116-117`），并用 `HighPrecisionTimer` 控制周期（`hs_exoskeleton.cpp:191-198`、`567-575`）。

**V2 简化：**

`ExoController` 使用 `std::chrono::steady_clock`、`Clock::dtSeconds()` 和 `std::this_thread::sleep_until`（`control/ExoController.cpp:46-69`、`120`）。

**判断：思路合理；初次评审时发现 `Clock.h` 缺少 `dtSeconds()` 声明。** 本轮已补 public 声明，clock test 已编译运行通过。

## 2. Eigen/矩阵依赖检查

### 2.1 V1 是否真的用了 Eigen？

**结论：V1 主控制逻辑本身没有显式做矩阵运算，但 V1 底层机器人封装确实使用 Eigen 保存角度/速度状态。**

证据：

- `hs_exoskeleton.cpp` 主文件没有直接包含 Eigen，也没有直接使用 `Eigen::Matrix`。
- `selfDevManuipulator.h` 包含 `<Eigen/Core>`，并用 `Matrix<double, DOF, 1>` 保存 `q/dq/sinq/cosq`（`include/Robot_H/selfDevManuipulator.h:11`、`43`）。
- `selfDevManuipulator::Ang()` / `AngVel()` 返回 Eigen 向量引用，V1 主循环通过 `motor_sys.Ang()[0]` 读取左右角度（`hs_exoskeleton.cpp:219-222`）。
- `TEXTIO.h`、`DataRecorder.h`、轨迹和 PID 头文件也包含 Eigen（如 `include/DataManager_H/TEXTIO.h:12`、`include/DataManager_H/DataRecorder.h:12`）。

### 2.2 V2 是否去掉了 Eigen？

**结论：V2 纯控制层去掉了 Eigen；V2 硬件目标仍间接依赖 Eigen。**

证据：

- `hs_exoskeleton_v2/control/`、`config/`、`logging/`、`tests/test_control.cpp` 未发现 Eigen/Matrix 使用。
- V2 硬件头 `hardware/KvaserExoHardware.h` 直接包含旧的 `selfDevManuipulator.h` 和 `motors.h`，并持有 `selfDevManuipulator<2> device_`（`hardware/KvaserExoHardware.h:10-33`）。这会把旧类中的 Eigen 依赖带入硬件目标。

**判断：当前简化是合理的折中。** 它让控制算法和纯测试摆脱 Eigen，同时不改底层电机控制。但如果目标是让完整 V2 可执行目标也不需要 Eigen，则必须替换或隔离 `selfDevManuipulator`，这会靠近底层控制边界，需谨慎。

## 3. 是否删除/遗漏了必要文件？

### 3.1 没有发现“误删底层电机控制文件”

V2 当前仍依赖旧项目中的底层文件，而不是复制/删除它们：

- AO：`../2026.4.14hs_exoskeleton/include/MultiHarmonicAO.h` 被 `control/PhaseEstimator.h` 使用。
- 单位转换：`../2026.4.14hs_exoskeleton/include/UnitConv.h` 被 `hardware/KvaserExoHardware.cpp` 使用。
- 电机/CAN：`../2026.4.14hs_exoskeleton/include/Kvaser_H/kvaser.h`、`include/Robot_H/selfDevManuipulator.h`、`include/Kvaser_H/motors.h` 被 V2 硬件层使用。
- V2 可执行目标在旧项目 `CMakeLists.txt` 中链接旧 Kvaser 源文件 `src/Kvaser_CPP/KvaserBase.cpp`、`src/Kvaser_CPP/kvaser.cpp`（`2026.4.14hs_exoskeleton/CMakeLists.txt:67-86`）。

### 3.2 曾存在的必要接口遗漏：`Clock::dtSeconds()`（已修复）

本地验证命令：

```bash
g++ -std=c++17 \
  -I2026.4.14hs_exoskeleton/include \
  -Ihs_exoskeleton_v2 \
  hs_exoskeleton_v2/tests/test_control.cpp \
  hs_exoskeleton_v2/control/IntentDetector.cpp \
  hs_exoskeleton_v2/control/TorqueProfile.cpp \
  hs_exoskeleton_v2/control/AssistStateMachine.cpp \
  hs_exoskeleton_v2/control/FreezeManager.cpp \
  hs_exoskeleton_v2/control/GaitFeatureExtractor.cpp \
  hs_exoskeleton_v2/control/PhaseEstimator.cpp \
  -o /tmp/hsx_v2_test_control && /tmp/hsx_v2_test_control
```

结果：`test_control_exit=0`。

本地验证命令：

```bash
g++ -std=c++17 \
  -Ihs_exoskeleton_v2 \
  hs_exoskeleton_v2/tests/test_clock.cpp \
  hs_exoskeleton_v2/logging/Clock.cpp \
  -o /tmp/hsx_v2_test_clock
```

修复前结果：编译失败，核心错误是：

```text
'class exo::Clock' has no member named 'dtSeconds'
no declaration matches 'double exo::Clock::dtSeconds()'
```

对应源码证据：

- `control/ExoController.cpp` 调用 `clock_.dtSeconds()`（`control/ExoController.cpp:63`、`67`）。
- `tests/test_clock.cpp` 调用 `clock.dtSeconds()`（`tests/test_clock.cpp:19`）。
- `logging/Clock.cpp` 定义 `double Clock::dtSeconds()`（`logging/Clock.cpp:26-31`）。
- `logging/Clock.h` 只有注释，没有 public 声明（`logging/Clock.h:14-20`）。

**判断：这不是应该删除的内容，而是 V2 简化/拆分时遗漏了必要声明。** 本轮已补 `Clock.h` 声明，不涉及底层电机控制。

### 3.3 V2 如果独立迁移，会缺少独立构建定义

V2 目录自身没有 `CMakeLists.txt`；构建目标定义在 `../2026.4.14hs_exoskeleton/CMakeLists.txt`（`2026.4.14hs_exoskeleton/CMakeLists.txt:42-86`）。

**判断：当前作为并行试验目录可以接受；如果 V2 要作为独立项目维护，应补独立构建文件。** 这同样不需要修改底层电机控制。

## 4. 简化是否合理：分项判断

| 简化项 | 合理性 | 风险等级 | 说明 |
| --- | --- | --- | --- |
| 主循环拆模块 | 合理 | 低 | 提升可读性与测试性。 |
| 参数从主文件搬到 config | 合理 | 低 | 修改阈值更集中。 |
| 控制层改用关节空间结构体 | 合理 | 低 | 通过 `IExoHardware` 保持硬件边界。 |
| GMM 改启发式意图检测 | 基本合理 | 中 | 更可读，但可能改变边界条件。 |
| 冻结逻辑独立为三态滞回 | 合理 | 低 | 行为更明确。 |
| 助力状态机显式 warm-up/ramp/fault | 合理 | 低 | 更安全、更可测。 |
| 力矩窗口改连续对称 cos | 方向合理但不等价 | 中 | 需要与 V1 日志比较相位窗口和力矩幅值。 |
| 二阶 Butterworth 改一阶低通 | 可接受但需验证 | 中 | 简化了滤波器，但相位延迟和峰值检测可能变化。 |
| 有符号 `diff_vel` 改为绝对速度 | 已做代码层修正，仍需实机验证 | 中 | 现在保留绝对速度给运动强度，同时用 `signed_phase_velocity_deg_s` 做峰/谷判断。 |
| 控制测试独立于硬件 | 合理 | 低 | 本地 `test_control` 编译运行通过。 |
| Clock 拆分 | 已修复接口遗漏 | 低 | 已声明 `dtSeconds()`，clock test 通过。 |

## 5. 仍有简化空间吗？

下面按“是否触碰底层电机控制”分级。

### 5.1 推荐优先做，不触碰底层电机控制

1. **补 `Clock::dtSeconds()` 头文件声明。**（已完成）
   - 范围：`hs_exoskeleton_v2/logging/Clock.h`。
   - 理由：V2 控制器和 clock test 都依赖该接口，属于必要声明遗漏。
   - 风险：低，不涉及硬件。

2. **恢复有符号相位速度，保留绝对速度作为运动强度。**（已完成代码层修正）
   - 范围：`GaitFeatures` 新增 `signed_phase_velocity_deg_s`，`phase_velocity_deg_s` 继续表示绝对运动强度。
   - 理由：V1 用有符号 `diff_vel` 判断峰/谷（`hs_exoskeleton.cpp:330-331`），V2 不应让锚点检测依赖绝对值速度。
   - 风险：中，不涉及底层电机，但影响控制算法，仍建议用实验日志验证。

3. **删除或下沉纯文档/注释之外的“未使用概念”。**
   - 例如确认 V2 是否还需要 `LoggingConfig::include_debug_columns`；当前配置里有该字段，但 logger 表头固定，未见使用。
   - 理由：减少误导性配置。
   - 风险：低。

4. **给 V2 增加独立 `CMakeLists.txt`。**
   - 理由：当前 V2 目标寄生在 V1 CMake 中；独立构建文件能明确哪些旧文件是外部依赖。
   - 风险：低到中，不改底层电机控制，但需正确引用旧 Kvaser/AO 文件。

5. **增加 V1/V2 力矩曲线对照测试或离线脚本。**
   - 理由：V2 `TorqueProfile` 与 V1 相位窗口不是等价重写，最好用相位扫描表证明差异是可接受的。
   - 风险：低，不上硬件。

6. **用假硬件实现 `IExoHardware` 做控制器集成测试。**
   - 理由：当前测试覆盖模块，未覆盖 `ExoController` 的完整循环、故障路径和 logger 调用。
   - 风险：低，不触碰底层电机。

### 5.2 可以考虑，但要谨慎隔离底层控制

1. **隐藏 `selfDevManuipulator` 对 V2 头文件使用者的传递依赖。**
   - 现状：`KvaserExoHardware.h` 直接包含 `selfDevManuipulator.h`，导致包含硬件头的编译单元也需要 Eigen、TEXTIO、Timer、canlib 等旧依赖。
   - 可选方向：使用 PIMPL 或将 legacy 设备成员放入 `.cpp` 私有实现中，使公共头只暴露 `IExoHardware`/`HardwareConfig`。
   - 注意：这改变的是依赖边界，不应改 `selfDevManuipulator`、`Kvaser` 的寄存器读写行为。

2. **把 `MultiHarmonicAO.h` 复制/封装到 V2 或作为显式共享模块。**
   - 现状：V2 `PhaseEstimator` 直接 include 旧项目头。
   - 好处：V2 独立性更强。
   - 风险：复制会造成双份 AO 实现漂移；更推荐共享模块或保留引用并在构建文件中明确。

### 5.3 不建议现在做，除非有实机验证计划

1. **替换 `selfDevManuipulator`，直接调用 Kvaser。**
   - 这可能真正移除完整 V2 目标的 Eigen 依赖，但会触碰底层电机控制边界。
   - 由于用户要求尽量不改底层电机控制，当前不建议优先做。

2. **重写 Kvaser 寄存器读写/力矩模式。**
   - 风险高，必须实机验证。

3. **大幅改动标定、使能、下电顺序。**
   - V2 当前顺序与 V1 基本对应：初始化 CAN、使能、零位、再次使能、控制、急停/下电（V1 见 `hs_exoskeleton.cpp:123-143`、`578-581`；V2 见 `KvaserExoHardware.cpp:16-47`、`98-108`）。不建议无实验数据时改。

## 6. V1 中确实存在的冗余/噪声

这些是 V2 简化时去掉或隔离的合理目标：

- `hs_exoskeleton.cpp` 中存在未实际使用或只在注释路径中使用的变量/概念：`Silence_time`（`hs_exoskeleton.cpp:38`）、`amp_deg`（`241`）、`dynamic_assist_gain` 当前只用于注释代码（`420-475`）。
- 力矩策略存在大量注释掉的旧分支（`hs_exoskeleton.cpp:431-475`），使实际行为不直观。
- PID、轨迹、DataRecorder 等旧通用工具未出现在 V1 当前主控制链路的直接逻辑中；V2 控制层不继续引入这些模块是合理的。
- V1 日志写入使用长参数列表（`hs_exoskeleton.cpp:547-553`），容易错位；V2 结构化 logger 更清晰。

## 7. 建议的后续顺序

1. ~~先补 `Clock::dtSeconds()` 声明，让 V2 clock test 和 `ExoController` 可编译。~~ 已完成。
2. ~~修正/澄清 `phase_velocity_deg_s` 的符号语义，至少让峰/谷检测不依赖绝对值速度。~~ 已完成代码层修正。
3. 用离线测试比较 V1/V2 在相同相位、频率、运动置信度下的力矩输出差异。
4. 增加假硬件集成测试，覆盖 `ExoController` 的 read failure、apply failure、freeze gate、allow_output gate。
5. 再考虑独立 CMake 和隐藏 legacy 硬件头依赖。
6. 暂不重写 `selfDevManuipulator`、`Kvaser`、CAN 寄存器控制、标定顺序和电机下电逻辑。

## 8. 本次验证记录

只读/轻量验证：

- 检查 V1/V2 CMake、入口、控制模块、硬件边界、日志和测试文件。
- 搜索 V2 中 Eigen/Matrix 使用：纯控制层未发现。
- 本地编译运行 V2 纯控制测试：通过。
- 本地编译运行 V2 clock test：通过。
- 修复前曾复现 clock test 编译失败，原因是 `Clock::dtSeconds()` 缺少头文件声明。

未验证：

- 未连接 Kvaser 硬件。
- 未编译完整 `hs_exoskeleton_v2` 硬件目标。
- 未对真实实验日志做 V1/V2 力矩、相位、冻结行为对齐。
