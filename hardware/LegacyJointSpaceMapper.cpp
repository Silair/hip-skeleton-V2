#include "hardware/LegacyJointSpaceMapper.h"

#include <algorithm>

#include "hardware/JointTypes.h"

namespace exo {

LegacyJointSpaceMapper::LegacyJointSpaceMapper(const HardwareConfig& config)
    : config_(config) {}

JointState LegacyJointSpaceMapper::toLeftJointState(double motor_position_rad, double motor_velocity_rad_s) const {
    JointState state{};
    state.raw_motor_position_rad = motor_position_rad;
    state.raw_motor_velocity_rad_s = motor_velocity_rad_s;
    // 左关节：电机角 × 位置比例、电机角速度 × 速度比例。
    state.position_rad = motor_position_rad * config_.left_joint_position_scale;
    state.velocity_rad_s = motor_velocity_rad_s * config_.left_joint_velocity_scale;
    return state;
}

JointState LegacyJointSpaceMapper::toRightJointState(double motor_position_rad, double motor_velocity_rad_s) const {
    JointState state{};
    state.raw_motor_position_rad = motor_position_rad;
    state.raw_motor_velocity_rad_s = motor_velocity_rad_s;
    state.position_rad = motor_position_rad * config_.right_joint_position_scale;
    state.velocity_rad_s = motor_velocity_rad_s * config_.right_joint_velocity_scale;
    return state;
}

double LegacyJointSpaceMapper::leftJointTorqueToMotorCommand(double joint_torque_nm) const {
    // 关节力矩 × 比例后，再按最大关节力矩对称夹紧，保护机械与驱动。
    return std::clamp(joint_torque_nm * config_.left_joint_to_motor_torque_scale,
        -config_.max_joint_torque_nm,
        config_.max_joint_torque_nm);
}

double LegacyJointSpaceMapper::rightJointTorqueToMotorCommand(double joint_torque_nm) const {
    return std::clamp(joint_torque_nm * config_.right_joint_to_motor_torque_scale,
        -config_.max_joint_torque_nm,
        config_.max_joint_torque_nm);
}

} // namespace exo
