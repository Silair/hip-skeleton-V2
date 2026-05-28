// 将每圈控制中间量写成 CSV：路径按日期与序号自动生成，析构时关闭文件。

#pragma once

#include <fstream>
#include <string>

#include "config/LoggingConfig.h"

namespace exo {

struct AssistOutput;
struct FreezeDecision;
struct GaitFeatures;
struct IntentEstimate;
struct PhaseEstimate;
struct TorqueCommand;
struct ExoState;

class ExoLogger {
public:
    explicit ExoLogger(const LoggingConfig& config);
    ~ExoLogger();

    bool open();
    const std::string& outputPath() const;
    // 写一行：状态 + 特征 + 相位 + 意图 + 冻结 + 助力 + 力矩指令。
    void write(const ExoState& state,
        const GaitFeatures& features,
        const PhaseEstimate& phase,
        const IntentEstimate& intent,
        const FreezeDecision& freeze,
        const AssistOutput& assist,
        const TorqueCommand& torque);
    void close();

private:
    std::string buildPath() const;

    LoggingConfig config_;
    std::ofstream stream_;
    std::string output_path_;
};

} // namespace exo
