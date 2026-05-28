// 基于项目内 legacy 封装（selfDevManuipulator / motors）的 Kvaser CAN 实现。
// 将寄存器读写封装为 IExoHardware，并在内部用 LegacyJointSpaceMapper 做关节空间映射。

#pragma once

#include "config/HardwareConfig.h"
#include "hardware/IExoHardware.h"
#include "hardware/LegacyJointSpaceMapper.h"

#include "selfDevManuipulator.h"
#include "motors.h"

namespace exo {

class KvaserExoHardware : public IExoHardware {
public:
    explicit KvaserExoHardware(const HardwareConfig& config);

    bool initialize() override;
    bool enable() override;
    bool calibrateZero() override;
    bool readState(ExoState& state) override;
    bool applyCommand(const ExoCommand& command) override;
    void emergencyStop() override;
    void shutdown() override;

private:
    // 读单轴位置(度)、速度(rpm)，失败时 error_code 非零。
    bool readMotorRegs(long motor_id, double& position_rad, double& velocity_rad_s, int& error_code) const;

    HardwareConfig config_;
    LegacyJointSpaceMapper mapper_;
    selfDevManuipulator<2> device_{};
    bool initialized_ = false;
    bool enabled_ = false;
};

} // namespace exo
