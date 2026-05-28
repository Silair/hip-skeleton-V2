// hs_exoskeleton_v2 程序入口：只做「组装配置 + 创建三大件 + 启停控制循环」。
// 具体控制逻辑在 control/ExoController；CAN/关节硬件在 hardware/；CSV 记录在 logging/。

#include <cstdlib>
#include <iostream>

#include "config/ControlConfig.h"
#include "config/HardwareConfig.h"
#include "config/LoggingConfig.h"
#include "control/ExoController.h"
#include "hardware/KvaserExoHardware.h"
#include "logging/ExoLogger.h"

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
