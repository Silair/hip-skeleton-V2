#include "logging/Clock.h"

namespace exo {

namespace {
int64_t toEpochMs(std::chrono::system_clock::time_point value) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(value.time_since_epoch()).count();
}
} // namespace

Clock::Clock()
    : steady_anchor_(SteadyClock::now()),
      last_dt_sample_(steady_anchor_),
      system_anchor_ms_(toEpochMs(SystemClock::now())) {}

int64_t Clock::epochMs() const {
    // 用 steady 的增量 + 构造时的 system 锚点，避免 steady 与墙钟混用带来的跳变问题。
    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(SteadyClock::now() - steady_anchor_).count();
    return system_anchor_ms_ + elapsed_ms;
}

double Clock::elapsedSeconds() const {
    return std::chrono::duration<double>(SteadyClock::now() - steady_anchor_).count();
}

double Clock::dtSeconds() {
    const auto now = SteadyClock::now();
    const double dt = std::chrono::duration<double>(now - last_dt_sample_).count();
    last_dt_sample_ = now;
    return dt;
}

} // namespace exo
