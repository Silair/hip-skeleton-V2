// 在「电机读数空间」与「控制用关节空间」之间做线性缩放与力矩限幅。
// 比例系数来自 HardwareConfig，与旧版外骨骼标定方式对齐。

#pragma once

#include "config/HardwareConfig.h"

namespace exo {

struct JointState;

class LegacyJointSpaceMapper {
public:
    explicit LegacyJointSpaceMapper(const HardwareConfig& config);

    JointState toLeftJointState(double motor_position_rad, double motor_velocity_rad_s) const;
    JointState toRightJointState(double motor_position_rad, double motor_velocity_rad_s) const;

    double leftJointTorqueToMotorCommand(double joint_torque_nm) const;
    double rightJointTorqueToMotorCommand(double joint_torque_nm) const;

private:
    HardwareConfig config_;
};

} // namespace exo
