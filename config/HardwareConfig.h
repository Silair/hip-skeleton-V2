// 硬件层可调参数：CAN 与左右电机 ID、关节↔电机缩放、力矩上限、上电标定行为。
// 由 KvaserExoHardware / LegacyJointSpaceMapper 读取；改这里即改默认硬件行为。

#pragma once

namespace exo {

struct HardwareConfig {
    // CAN 口与速率（与 Kvaser 驱动约定一致，常用 1 Mbps）。
    int can_channel = 0;
    int can_bitrate = 1000000;
    // 左右电机在总线上的节点 ID。
    long left_motor_id = 0x0001;
    long right_motor_id = 0x0002;

    // 电机编码器空间 → 逻辑「关节」位置/速度的线性比例（兼容旧系统标定）。
    double left_joint_position_scale = 1.08;
    double right_joint_position_scale = 0.83;
    double left_joint_velocity_scale = 1.08;
    double right_joint_velocity_scale = 0.83;

    // 关节力矩 (N·m) → 下发给驱动的力矩指令比例，以及关节侧最大力矩夹紧。
    double left_joint_to_motor_torque_scale = 1.0;
    double right_joint_to_motor_torque_scale = 1.0;
    double max_joint_torque_nm = 8.0;

    // 上电后是否自动零位、等待用户准备与零位后的短暂延时（毫秒）。
    bool calibrate_on_start = true;
    bool ignore_motor_enable_result = false;
    bool ignore_zero_result = false;
    int calibration_prompt_wait_ms = 1000;
    int post_zero_wait_ms = 200;
};

} // namespace exo
