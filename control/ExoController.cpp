// ExoController：V2 实时控制环的「编排层」。
// 不负责具体算法内部实现，只按固定顺序调用各子模块，并把 ExoState 在环与环之间传递。
// 阅读顺序建议：本文件 run() 主循环 → 各 *.h 中 struct 字段含义 → ControlConfig 里对应子配置。

#include "control/ExoController.h"

#include "control/GaitFeatures.h"
#include "hardware/JointTypes.h"

#include <chrono>
#include <thread>
#include <algorithm>
#include <iostream>

namespace exo {

namespace {

const char* assistStateName(AssistState state) {
    switch (state) {
    case AssistState::Transparent:
        return "Transparent";
    case AssistState::Tracking:
        return "Tracking";
    case AssistState::Ramp:
        return "Ramp";
    case AssistState::Active:
        return "Active";
    case AssistState::Stopping:
        return "Stopping";
    case AssistState::Frozen:
        return "Frozen";
    case AssistState::Fault:
        return "Fault";
    }
    return "Unknown";
}

} // namespace

// 成员初始化顺序 = 声明顺序（C++ 规则）。各子模块在构造时从 config_ 的不同子结构取参：
// - phase：特征提取与相位估计共用一套步态/相位相关阈值与时间常数。
// - intent / freeze / assist / torque：各自独立子配置。
// hardware_、logger_ 仅保存引用，寿命须由 main 保证长于 ExoController。
ExoController::ExoController(const ControlConfig& config, IExoHardware& hardware, ExoLogger& logger)
    : config_(config),
      hardware_(hardware),
      logger_(logger),
      feature_extractor_(config.phase),
      phase_estimator_(config.phase),
      intent_detector_(config.intent),
      stop_detector_(config.stop),
      freeze_manager_(config.freeze),
      assist_state_machine_(config.assist),
      torque_profile_(config.torque),
      stop_torque_limiter_(config.stop) {}

// 一次性上电与日志准备；任一步失败则 run() 不应再调用（main 里会走 shutdown）。
// 顺序：总线/设备 → 电机使能 → 零位标定（若实现支持跳过则由硬件层决定）→ 打开日志文件。
bool ExoController::initialize() {
    if (!hardware_.initialize()) {
        std::cerr << "ExoController initialize failed: hardware initialize failed" << std::endl;
        return false;
    }
    if (!hardware_.enable()) {
        std::cerr << "ExoController initialize failed: motor enable failed" << std::endl;
        return false;
    }
    if (!hardware_.calibrateZero()) {
        std::cerr << "ExoController initialize failed: zero calibration failed" << std::endl;
        return false;
    }
    if (!logger_.open()) {
        std::cerr << "ExoController initialize failed: logger open failed" << std::endl;
        return false;
    }
    std::cout << "ExoController initialized. Log: " << logger_.outputPath() << std::endl;
    return true;
}

// 固定目标频率的「软实时」循环：用 steady_clock + sleep_until 对齐节拍，实际周期仍受 OS 调度影响。
// measured_dt_s 反映真实间隔；controller_dt_s 对其限幅，避免偶发长卡顿把积分/滤波一步拉爆。
bool ExoController::run() {
    using clock = std::chrono::steady_clock;
    const double target_dt_s = 1.0 / config_.loop_frequency_hz;
    auto next_tick = clock::now();

    // 进入循环前先读一帧：让 state 非空，后续 feature/phase 首帧行为更确定；失败则直接返回（与环内读失败不同，环内会急停以防已下发力矩）。
    ExoState state{};
    if (!hardware_.readState(state)) {
        return false;
    }

    double current_time_s = 0.0;
    double next_status_print_s = 0.0;
    uint64_t loop_seq = 0;
    FreezeDecision freeze{};
    // 丢弃「从构造到进入循环」的间隔，避免第一帧 dt 巨大；此后每帧 dtSeconds() 为与上一采样点间隔。
    clock_.dtSeconds();
    while (current_time_s < config_.run_duration_s) {
        // 节拍推进：不依赖上一帧实际耗时累加，而是按理想网格 next_tick += T，长期更不易漂移。
        next_tick += std::chrono::duration_cast<clock::duration>(std::chrono::duration<double>(target_dt_s));
        const double measured_dt_s = clock_.dtSeconds();
        // 防止 OS 调度抖动导致 dt 极端值，把控制用 dt 夹在合理范围。
        const double controller_dt_s = std::clamp(measured_dt_s, 0.0001, 0.1);
        current_time_s = clock_.elapsedSeconds();
        // 先由控制器写入本周期时间戳与序号；随后 readState 填充关节量、健康位等（若实现会保留未覆盖字段）。
        state.time_s = current_time_s;
        state.loop_seq = ++loop_seq;
        state.epoch_ms = clock_.epochMs();
        state.monotonic_time_s = current_time_s;
        state.dt_s = measured_dt_s;

        if (!hardware_.readState(state)) {
            hardware_.emergencyStop();
            return false;
        }

        // ---------- 控制链：传感器态 → 停步检测/步态特征 → 相位（可受冻结/停步策略门控）----------
        StopDecision stop = stop_detector_.update(state, controller_dt_s);
        GaitFeatures features = feature_extractor_.update(state, controller_dt_s);
        const bool phase_tracking_enabled = freeze.phase_tracking_enabled && stop.phase_tracking_enabled;
        PhaseEstimate phase = phase_estimator_.update(
            features,
            controller_dt_s,
            phase_tracking_enabled,
            last_assist_state_,
            last_stop_probability_,
            last_motion_confidence_,
            freeze.freeze_requested,
            stop.stop_requested);
        // 将相位估计的幅值/频率写回特征，供意图检测使用更一致的观测。
        features.amplitude_rad = phase.amplitude_rad;
        features.frequency_hz = phase.frequency_hz;

        // ---------- 意图与冻结：输出 freeze 影响后续相位门控与最终力矩是否允许 ----------
        IntentEstimate intent = intent_detector_.update(features, controller_dt_s);
        last_stop_probability_ = intent.stop_probability;
        last_motion_confidence_ = intent.motion_confidence;
        freeze = stop.stop_requested ? freeze_manager_.resetToLive() : freeze_manager_.update(intent, controller_dt_s);

        // ---------- 助力状态机：综合运动置信度、相位有效、锚点、冻结请求与健康 ----------
        AssistInputs assist_inputs{};
        assist_inputs.motion_confidence = intent.motion_confidence;
        assist_inputs.phase_valid = phase.valid && !freeze.recovery_active && !stop.stop_requested;
        assist_inputs.anchor_detected = phase.anchor_detected;
        assist_inputs.stop_requested = stop.stop_requested;
        assist_inputs.freeze_requested = freeze.freeze_requested;
        assist_inputs.faulted = !state.healthy;

        AssistOutput assist = assist_state_machine_.update(assist_inputs, controller_dt_s);
        last_assist_state_ = assist.state;
        // 正常助力按相位生成；自然停步期间只撤掉上一帧力矩，不再生成新的相位力矩峰。
        TorqueCommand torque = assist.state == AssistState::Stopping
            ? stop_torque_limiter_.update(previous_torque_, controller_dt_s)
            : torque_profile_.compute(phase.phase_rad, phase.frequency_hz, assist.torque_scale, assist.allow_output);
        previous_torque_ = torque;

        // ---------- 组指令：关节力矩 + 总门控（冻结/故障时禁止驱动输出）----------
        ExoCommand command{};
        command.left_joint_torque_nm = torque.left_nm;
        command.right_joint_torque_nm = torque.right_nm;
        command.allow_output = assist.allow_output && !freeze.freeze_requested && state.healthy;

        if (current_time_s >= next_status_print_s) {
            std::cout << "t=" << current_time_s
                      << "s state=" << assistStateName(assist.state)
                      << " allow_output=" << (command.allow_output ? 1 : 0)
                      << " torque_scale=" << assist.torque_scale
                      << " motion_conf=" << intent.motion_confidence
                      << " stop_prob=" << intent.stop_probability
                      << " phase_valid=" << (phase.valid ? 1 : 0)
                      << " anchor=" << (phase.anchor_detected ? 1 : 0)
                      << " left_nm=" << torque.left_nm
                      << " right_nm=" << torque.right_nm
                      << std::endl;
            next_status_print_s = current_time_s + 1.0;
        }

        // 先写 CSV 再下发：记录的是「本周期算出的意图/力矩」；若 apply 失败会急停并返回，本行已执行便于事后对齐最后一帧 good 数据。
        logger_.write(state, features, phase, intent, freeze, assist, torque);

        if (!hardware_.applyCommand(command)) {
            hardware_.emergencyStop();
            return false;
        }

        std::this_thread::sleep_until(next_tick);
    }

    return true;
}

// 先关日志再关硬件：尽量 flush 文件；硬件 shutdown 可能较慢或阻塞。
void ExoController::shutdown() {
    logger_.close();
    hardware_.shutdown();
}

} // namespace exo
