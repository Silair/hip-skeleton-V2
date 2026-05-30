// hs_exoskeleton_v2 程序入口：只做「组装配置 + 创建三大件 + 启停控制循环」。
// 具体控制逻辑在 control/ExoController；CAN/关节硬件在 hardware/；CSV 记录在 logging/。

#include <cstdlib>
#include <iostream>
#include <string>

#include "config/ControlConfig.h"
#include "config/HardwareConfig.h"
#include "config/LoggingConfig.h"
#include "control/ExoController.h"
#include "hardware/KvaserExoHardware.h"
#include "logging/ExoLogger.h"

namespace {

bool parseDoubleEnv(const char* name, double min_value, double max_value, double& target) {
    const char* raw = std::getenv(name);
    if (!raw || raw[0] == '\0') {
        return false;
    }
    char* end = nullptr;
    const double parsed = std::strtod(raw, &end);
    if (end == raw || parsed < min_value || parsed > max_value) {
        std::cerr << "Ignoring invalid " << name << "=" << raw
                  << " (expected " << min_value << ".." << max_value << ")" << std::endl;
        return false;
    }
    target = parsed;
    std::cout << name << "=" << target << std::endl;
    return true;
}

bool parseIntEnv(const char* name, long min_value, long max_value, long& target) {
    const char* raw = std::getenv(name);
    if (!raw || raw[0] == '\0') {
        return false;
    }
    char* end = nullptr;
    const long parsed = std::strtol(raw, &end, 0);
    if (end == raw || parsed < min_value || parsed > max_value) {
        std::cerr << "Ignoring invalid " << name << "=" << raw
                  << " (expected " << min_value << ".." << max_value << ")" << std::endl;
        return false;
    }
    target = parsed;
    std::cout << name << "=" << target << std::endl;
    return true;
}

bool parseBoolEnv(const char* name, bool& target) {
    const char* raw = std::getenv(name);
    if (!raw || raw[0] == '\0') {
        return false;
    }
    const std::string value(raw);
    if (value == "1" || value == "true" || value == "TRUE" || value == "yes" || value == "YES") {
        target = true;
    } else if (value == "0" || value == "false" || value == "FALSE" || value == "no" || value == "NO") {
        target = false;
    } else {
        std::cerr << "Ignoring invalid " << name << "=" << raw << " (expected true/false or 1/0)" << std::endl;
        return false;
    }
    std::cout << name << "=" << (target ? "true" : "false") << std::endl;
    return true;
}

void applyEnvironmentOverrides(
    exo::HardwareConfig& hardware_config,
    exo::ControlConfig& control_config,
    exo::LoggingConfig& logging_config) {
    parseDoubleEnv("HSX_RUN_DURATION_S", 0.1, 600.0, control_config.run_duration_s);
    parseDoubleEnv("HSX_LOOP_FREQUENCY_HZ", 10.0, 200.0, control_config.loop_frequency_hz);

    double torque_limit_nm = control_config.torque.max_torque_nm;
    if (parseDoubleEnv("HSX_MAX_TORQUE_NM", 0.0, 8.0, torque_limit_nm)) {
        control_config.torque.max_torque_nm = torque_limit_nm;
        hardware_config.max_joint_torque_nm = torque_limit_nm;
    }
    parseDoubleEnv("HSX_MAX_JOINT_TORQUE_NM", 0.0, 8.0, hardware_config.max_joint_torque_nm);
    parseDoubleEnv("HSX_LEFT_TORQUE_SCALE", -2.0, 2.0, hardware_config.left_joint_to_motor_torque_scale);
    parseDoubleEnv("HSX_RIGHT_TORQUE_SCALE", -2.0, 2.0, hardware_config.right_joint_to_motor_torque_scale);
    parseDoubleEnv("HSX_BASE_GAIN_MIN_NM", 0.0, 8.0, control_config.torque.base_gain_min_nm);
    parseDoubleEnv("HSX_BASE_GAIN_MAX_NM", 0.0, 8.0, control_config.torque.base_gain_max_nm);
    parseDoubleEnv("HSX_LEAD_ANGLE_RAD", -1.0, 1.0, control_config.torque.lead_angle_rad);
    parseDoubleEnv("HSX_RAMP_UP_RATE_PER_S", 0.05, 2.0, control_config.assist.ramp_up_rate_per_s);
    parseDoubleEnv("HSX_RAMP_DOWN_RATE_PER_S", 0.1, 5.0, control_config.assist.ramp_down_rate_per_s);
    parseDoubleEnv("HSX_MOTION_ENTRY_CONFIDENCE", 0.0, 1.0, control_config.assist.motion_entry_confidence);
    parseDoubleEnv("HSX_MOTION_EXIT_CONFIDENCE", 0.0, 1.0, control_config.assist.motion_exit_confidence);
    long warmup_anchor_count = control_config.assist.warmup_anchor_count;
    if (parseIntEnv("HSX_WARMUP_ANCHOR_COUNT", 0, 10, warmup_anchor_count)) {
        control_config.assist.warmup_anchor_count = static_cast<int>(warmup_anchor_count);
    }
    parseDoubleEnv("HSX_AO_INITIAL_FREQUENCY_HZ", 0.3, 1.6, control_config.phase.ao_initial_frequency_hz);
    parseDoubleEnv("HSX_AO_MIN_FREQUENCY_HZ", 0.3, 1.6, control_config.phase.ao_min_frequency_hz);
    parseDoubleEnv("HSX_AO_MAX_FREQUENCY_HZ", 0.3, 1.6, control_config.phase.ao_max_frequency_hz);
    parseDoubleEnv("HSX_ANCHOR_FREQUENCY_GAIN", 0.0, 1.0, control_config.phase.anchor_frequency_gain);
    parseDoubleEnv("HSX_ANCHOR_FREQUENCY_GAIN_RAMP", 0.0, 1.0, control_config.phase.anchor_frequency_gain_ramp);
    parseDoubleEnv("HSX_ANCHOR_FREQUENCY_MAX_HZ", 0.3, 1.6, control_config.phase.anchor_frequency_max_hz);
    parseDoubleEnv("HSX_STOP_VELOCITY_THRESHOLD_RAD_S", 0.01, 0.6, control_config.stop.velocity_threshold_rad_s);
    parseDoubleEnv("HSX_STOP_ENTER_HOLD_SECONDS", 0.02, 1.0, control_config.stop.enter_hold_seconds);
    parseDoubleEnv("HSX_FREEZE_ENTER_STOP_PROBABILITY", 0.1, 0.95, control_config.freeze.enter_stop_probability);
    parseDoubleEnv("HSX_FREEZE_ENTER_HOLD_SECONDS", 0.02, 1.0, control_config.freeze.enter_hold_seconds);

    long can_channel = hardware_config.can_channel;
    if (parseIntEnv("HSX_CAN_CHANNEL", 0, 16, can_channel)) {
        hardware_config.can_channel = static_cast<int>(can_channel);
    }
    long left_motor_id = hardware_config.left_motor_id;
    if (parseIntEnv("HSX_LEFT_MOTOR_ID", 0, 0x7fffffffL, left_motor_id)) {
        hardware_config.left_motor_id = left_motor_id;
    }
    long right_motor_id = hardware_config.right_motor_id;
    if (parseIntEnv("HSX_RIGHT_MOTOR_ID", 0, 0x7fffffffL, right_motor_id)) {
        hardware_config.right_motor_id = right_motor_id;
    }
    parseBoolEnv("HSX_CALIBRATE_ON_START", hardware_config.calibrate_on_start);
    parseBoolEnv("HSX_IGNORE_MOTOR_ENABLE_RESULT", hardware_config.ignore_motor_enable_result);
    parseBoolEnv("HSX_IGNORE_ZERO_RESULT", hardware_config.ignore_zero_result);

    if (const char* log_base = std::getenv("HSX_LOG_BASE_FOLDER")) {
        logging_config.base_folder = log_base;
        std::cout << "HSX_LOG_BASE_FOLDER=" << logging_config.base_folder << std::endl;
    }
    if (const char* log_prefix = std::getenv("HSX_LOG_PREFIX")) {
        logging_config.filename_prefix = log_prefix;
        std::cout << "HSX_LOG_PREFIX=" << logging_config.filename_prefix << std::endl;
    }
}

} // namespace

int main() {
    // 三套默认配置（结构体在 config/*.h）；需要改参数时改头文件或后续可改为读文件。
    exo::HardwareConfig hardware_config{};
    exo::ControlConfig control_config{};
    exo::LoggingConfig logging_config{};

    // 可选环境变量：给日志打上「同步会话 / 数据流」标识，便于多机或多次实验对齐分析。
    if (const char* sync_session_id = std::getenv("HSX_SYNC_SESSION_ID")) {
        logging_config.sync_session_id = sync_session_id;
    }
    if (const char* stream_id = std::getenv("HSX_STREAM_ID")) {
        logging_config.stream_id = stream_id;
    }
    applyEnvironmentOverrides(hardware_config, control_config, logging_config);

    // 依赖顺序：硬件与日志器先构造，控制器持有二者的引用（不负责释放它们）。
    exo::KvaserExoHardware hardware(hardware_config);
    exo::ExoLogger logger(logging_config);
    exo::ExoController controller(control_config, hardware, logger);

    // initialize：打开设备、标定、状态机等一次性准备；失败则仍走 shutdown 做清理。
    if (!controller.initialize()) {
        std::cerr << "Failed to initialize hs_exoskeleton_v2" << std::endl;
        controller.shutdown();
        return 1;
    }

    // run：主控制循环（读传感器 → 算力矩/指令 → 写执行器），正常结束或出错都会返回 bool。
    const bool ok = controller.run();
    controller.shutdown();
    if (!logger.outputPath().empty()) {
        std::cout << "hs_exoskeleton_v2 log: " << logger.outputPath() << std::endl;
        std::cout << "Analyze with: python3 hs_exoskeleton_v2/tools/analyze_run.py "
                  << logger.outputPath() << std::endl;
    }
    return ok ? 0 : 1;
}
