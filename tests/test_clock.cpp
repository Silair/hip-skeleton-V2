// 单元测试：验证 Clock 的 epoch 单调递增、elapsed 增长、dt 非负（不依赖硬件）。

#include <cassert>
#include <chrono>
#include <thread>

#include "logging/Clock.h"

int main() {
    exo::Clock clock;
    const auto first_epoch = clock.epochMs();
    const auto first_elapsed = clock.elapsedSeconds();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    const auto second_epoch = clock.epochMs();
    const auto second_elapsed = clock.elapsedSeconds();

    assert(second_epoch >= first_epoch);
    assert(second_elapsed > first_elapsed);
    assert(clock.dtSeconds() >= 0.0);
    return 0;
}
