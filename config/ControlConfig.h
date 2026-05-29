// 控制环全部超参：意图、冻结、助力状态机、力矩形状、相位估计等，由 ExoController 分发给各子模块。

#pragma once

namespace exo {

// 从步态特征推断「像在走」还是「像停住」的阈值与平滑系数（IntentDetector）。
struct IntentConfig {
    double low_motion_spread_deg = 4.0;
    double active_motion_spread_deg = 24.0;
    double low_motion_velocity_deg_s = 8.0;
    double active_motion_velocity_deg_s = 80.0;
    double low_motion_amplitude_rad = 0.05;
    double active_motion_amplitude_rad = 0.45;
    double low_motion_frequency_hz = 0.10;
    double active_motion_frequency_hz = 0.85;
    double smoothing_alpha = 0.20;
};

// 进入/退出「冻结」的滞回：概率阈值 + 持续满足时间，避免抖动（FreezeManager）。
struct StopConfig {
    double velocity_threshold_rad_s = 0.08;
    double exit_velocity_threshold_rad_s = 0.16;
    double enter_hold_seconds = 0.16;
    double exit_hold_seconds = 0.08;
    double velocity_filter_alpha = 0.25;
};

struct FreezeConfig {
    double enter_stop_probability = 0.78;
    double exit_stop_probability = 0.35;
    double resume_motion_confidence = 0.55;
    double enter_hold_seconds = 0.20;
    double resume_hold_seconds = 0.20;
};

// 助力从透明到全开的门限与爬升/下降速率（AssistStateMachine）。
struct AssistConfig {
    int warmup_anchor_count = 3;
    double motion_entry_confidence = 0.45;
    double motion_exit_confidence = 0.25;
    double ramp_up_rate_per_s = 0.60;
    double ramp_down_rate_per_s = 1.20;
};

// 按估算步频映射基础增益、相位超前角与力矩上限（TorqueProfile）。
struct TorqueConfig {
    double base_gain_min_nm = 3.0;
    double base_gain_max_nm = 6.0;
    double gain_min_frequency_hz = 0.6;
    double gain_max_frequency_hz = 1.2;
    double lead_angle_rad = 0.20;
    double max_torque_nm = 8.0;
};

// 自适应振荡器 AO 的频率范围、低通、峰值锚点条件等（PhaseEstimator / GaitFeatureExtractor）。
struct PhaseConfig {
    double target_frequency_hz = 50.0;
    double ao_initial_frequency_hz = 0.8;
    double ao_min_frequency_hz = 0.6;
    double ao_max_frequency_hz = 1.4;
    double low_pass_cutoff_hz = 8.0;
    double peak_min_spread_deg = 18.0;
    double anchor_min_fraction_of_period = 0.40;
    double anchor_decay_gain = 1.0;
    double ao_internal_step_s = 0.002;
};

// 总控：主循环频率、单次运行时长，以及上面各子配置嵌套结构体。
struct ControlConfig {
    double loop_frequency_hz = 50.0;
    double run_duration_s = 180.0;
    IntentConfig intent{};
    StopConfig stop{};
    FreezeConfig freeze{};
    AssistConfig assist{};
    TorqueConfig torque{};
    PhaseConfig phase{};
};

} // namespace exo
