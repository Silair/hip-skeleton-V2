// 日志输出路径与 CSV 列语义相关标识；ExoLogger 按日期分子目录、按序号避免覆盖。

#pragma once

#include <string>

namespace exo {

struct LoggingConfig {
    // 根目录下按日期建子文件夹，例如 Data/2026.05.06/。
    std::string base_folder = "Data";
    // 文件名前缀，实际文件形如 prefix_1_2026.05.06.csv。
    std::string filename_prefix = "AO_v2_test";
    // 多设备/多轮实验对齐用会话 ID，写入 CSV 首列附近。
    std::string sync_session_id = "sync_local_exo";
    // 数据流标识（例如不同算法版本）。
    std::string stream_id = "exo_hs_v2";
    // 预留：是否在 CSV 中增加调试列（当前表头实现以 ExoLogger 为准）。
    bool include_debug_columns = true;
};

} // namespace exo
