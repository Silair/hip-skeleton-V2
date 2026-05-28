// 硬件抽象接口：控制层只依赖本接口，便于测试桩或替换为其他总线实现。

#pragma once

namespace exo {

struct ExoCommand;
struct ExoState;

class IExoHardware {
public:
    virtual ~IExoHardware() = default;

    // 打开总线/设备，一次性调用。
    virtual bool initialize() = 0;
    // 上电使能电机。
    virtual bool enable() = 0;
    // 将当前姿态记为零位（可选由配置关闭）。
    virtual bool calibrateZero() = 0;
    // 填充 state；返回 false 表示本轮读失败（如通信错误）。
    virtual bool readState(ExoState& state) = 0;
    // 将关节力矩指令发到驱动（单位与实现约定一致）。
    virtual bool applyCommand(const ExoCommand& command) = 0;
    // 急停：立即卸力或抱闸等，由控制环在故障路径调用。
    virtual void emergencyStop() = 0;
    // 释放资源、下电。
    virtual void shutdown() = 0;
};

} // namespace exo
