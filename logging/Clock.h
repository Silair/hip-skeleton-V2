// 为控制环与日志提供统一时间：墙钟近似 epoch 毫秒、从启动起的秒数、相邻两次调用的 dt。

#pragma once

#include <chrono>
#include <cstdint>

namespace exo {

class Clock {
public:
    Clock();

    // 启动时刻对齐后的「近似 Unix 毫秒」，便于与外部系统对齐。
    int64_t epochMs() const;
    // 从构造时刻起的单调经过时间（秒）。
    double elapsedSeconds() const;
    // 相邻两次调用之间的时间间隔（秒），首次调用返回自构造起的间隔。
    double dtSeconds();

private:
    using SteadyClock = std::chrono::steady_clock;
    using SystemClock = std::chrono::system_clock;

    SteadyClock::time_point steady_anchor_;
    SteadyClock::time_point last_dt_sample_;
    int64_t system_anchor_ms_ = 0;
};

} // namespace exo
