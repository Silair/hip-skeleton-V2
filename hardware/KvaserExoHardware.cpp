#include "hardware/KvaserExoHardware.h"

#include <chrono>
#include <iostream>
#include <thread>

#include "UnitConv.h"
#include "kvaser.h"
#include "hardware/JointTypes.h"

namespace exo {

KvaserExoHardware::KvaserExoHardware(const HardwareConfig& config)
    : config_(config),
      mapper_(config) {}

bool KvaserExoHardware::initialize() {
    MotorParameters motor;
    device_.setMotor(motor);
    // 将配置中的比特率映射到 Kvaser API 常量（1M 用枚举，其余按数值）。
    const int bitrate = (config_.can_bitrate == 1000000) ? canBITRATE_1M : config_.can_bitrate;
    const int init_result = device_.CANBus().canInit(config_.can_channel, bitrate);
    initialized_ = (init_result == 0);
    std::cout << "Kvaser initialize: channel=" << config_.can_channel
              << " bitrate=" << config_.can_bitrate
              << " result=" << init_result << std::endl;
    return initialized_;
}

bool KvaserExoHardware::enable() {
    if (!initialized_) {
        return false;
    }

    const int left_result = device_.CANBus().MotorOnOff(config_.left_motor_id, 0x01);
    const int right_result = device_.CANBus().MotorOnOff(config_.right_motor_id, 0x01);
    const bool left_ok = left_result == 0;
    const bool right_ok = right_result == 0;
    std::cout << "Motor enable: left_id=" << config_.left_motor_id
              << " result=" << left_result
              << ", right_id=" << config_.right_motor_id
              << " result=" << right_result << std::endl;
    if ((!left_ok || !right_ok) && config_.ignore_motor_enable_result) {
        std::cerr << "WARNING: ignoring MotorOnOff failure because HSX_IGNORE_MOTOR_ENABLE_RESULT=1" << std::endl;
        enabled_ = true;
        return true;
    }
    enabled_ = left_ok && right_ok;
    return enabled_;
}

bool KvaserExoHardware::calibrateZero() {
    // 未初始化或未使能时直接返回当前能否继续；关闭 calibrate_on_start 时跳过零位仅返回已使能。
    if (!initialized_ || !enabled_ || !config_.calibrate_on_start) {
        return initialized_ && enabled_;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(config_.calibration_prompt_wait_ms));
    const int left_result = device_.CANBus().MotorZeroSet(config_.left_motor_id);
    const int right_result = device_.CANBus().MotorZeroSet(config_.right_motor_id);
    const bool left_ok = left_result == 0;
    const bool right_ok = right_result == 0;
    std::cout << "Motor zero set: left_result=" << left_result
              << ", right_result=" << right_result << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(config_.post_zero_wait_ms));
    if ((!left_ok || !right_ok) && !config_.ignore_zero_result) {
        return false;
    }
    if (!left_ok || !right_ok) {
        std::cerr << "WARNING: ignoring MotorZeroSet failure because HSX_IGNORE_ZERO_RESULT=1" << std::endl;
    }
    return enable();
}

bool KvaserExoHardware::readMotorRegs(long motor_id, double& position_rad, double& velocity_rad_s, int& error_code) const {
    float position_deg = 0.0F;
    float velocity_rpm = 0.0F;
    const bool position_ok = device_.CANBus().MotorGetReg(motor_id, MOTOR_REG_POSITION_MEAS_DEGREE, reinterpret_cast<uint32_t*>(&position_deg)) == 0;
    const bool velocity_ok = device_.CANBus().MotorGetReg(motor_id, MOTOR_REG_SPEED_MEAS_RPM, reinterpret_cast<uint32_t*>(&velocity_rpm)) == 0;
    if (!position_ok || !velocity_ok) {
        error_code = 1;
        return false;
    }

    position_rad = DEG2RAD(position_deg);
    velocity_rad_s = RPM2RAD(velocity_rpm);
    error_code = 0;
    return true;
}

bool KvaserExoHardware::readState(ExoState& state) {
    state.enabled = enabled_;

    double left_pos_rad = 0.0;
    double left_vel_rad_s = 0.0;
    double right_pos_rad = 0.0;
    double right_vel_rad_s = 0.0;
    int left_error = 0;
    int right_error = 0;

    const bool left_ok = readMotorRegs(config_.left_motor_id, left_pos_rad, left_vel_rad_s, left_error);
    const bool right_ok = readMotorRegs(config_.right_motor_id, right_pos_rad, right_vel_rad_s, right_error);
    if (!left_ok || !right_ok) {
        std::cerr << "Motor read failed: left_ok=" << left_ok
                  << " right_ok=" << right_ok << std::endl;
    }

    // 读成功后再映射到关节空间，便于上层算法统一处理。
    state.left = mapper_.toLeftJointState(left_pos_rad, left_vel_rad_s);
    state.right = mapper_.toRightJointState(right_pos_rad, right_vel_rad_s);
    state.error_left = left_error;
    state.error_right = right_error;
    state.healthy = left_ok && right_ok;
    state.health = state.healthy ? ExoHealth::Nominal : ExoHealth::CommunicationFault;
    return state.healthy;
}

bool KvaserExoHardware::applyCommand(const ExoCommand& command) {
    // allow_output 为 false 时强制发零力矩，避免状态机间隙误出力。
    const double left_command = command.allow_output ? mapper_.leftJointTorqueToMotorCommand(command.left_joint_torque_nm) : 0.0;
    const double right_command = command.allow_output ? mapper_.rightJointTorqueToMotorCommand(command.right_joint_torque_nm) : 0.0;

    const int left_result = device_.CANBus().torqueMode(config_.left_motor_id, static_cast<float>(left_command));
    const int right_result = device_.CANBus().torqueMode(config_.right_motor_id, static_cast<float>(right_command));
    const bool left_ok = left_result == 0;
    const bool right_ok = right_result == 0;
    if (!left_ok || !right_ok) {
        std::cerr << "Torque command failed: left_result=" << left_result
                  << " right_result=" << right_result << std::endl;
    }
    return left_ok && right_ok;
}

void KvaserExoHardware::emergencyStop() {
    device_.brake();
}

void KvaserExoHardware::shutdown() {
    emergencyStop();
    if (initialized_) {
        device_.CANBus().MotorOnOff(config_.left_motor_id, 0x00);
        device_.CANBus().MotorOnOff(config_.right_motor_id, 0x00);
    }
    enabled_ = false;
}

} // namespace exo
