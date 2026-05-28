// 控制与硬件共用的「关节空间」状态/指令/健康枚举；与 CAN 寄存器细节解耦。

#pragma once

#include <cstdint>

namespace exo {

// 设备级健康：通信失败与驱动故障等（当前实现主要用 Nominal / CommunicationFault）。
enum class ExoHealth {
    Nominal,
    CommunicationFault,
    DriverFault,
};

// 单侧关节：逻辑量（经缩放后）与原始电机侧量，便于对照调试。
struct JointState {
    double position_rad = 0.0;
    double velocity_rad_s = 0.0;
    double raw_motor_position_rad = 0.0;
    double raw_motor_velocity_rad_s = 0.0;
};

// 整机关节反馈快照：时间戳、左右关节、使能与错误码。
struct ExoState {
    double time_s = 0.0;
    uint64_t loop_seq = 0;
    int64_t epoch_ms = 0;
    double monotonic_time_s = 0.0;
    double dt_s = 0.0;
    JointState left{};
    JointState right{};
    bool enabled = false;
    bool healthy = false;
    int error_left = 0;
    int error_right = 0;
    ExoHealth health = ExoHealth::Nominal;
};

// 下发给硬件的关节力矩指令；allow_output 为 false 时硬件侧应不发有效力矩。
struct ExoCommand {
    double left_joint_torque_nm = 0.0;
    double right_joint_torque_nm = 0.0;
    bool allow_output = false;
};

} // namespace exo
