#include "watchdog.h"

namespace Watchdog {
namespace {
    volatile uint32_t s_last_heartbeat_ms[static_cast<size_t>(TaskId::COUNT)] = {0};
}

void Init() {
    uint32_t now = millis();
    for (auto &t : s_last_heartbeat_ms) t = now;
}

void ReportHeartbeat(TaskId id) {
    s_last_heartbeat_ms[static_cast<size_t>(id)] = millis();
}

uint32_t MsSinceLastHeartbeat(TaskId id) {
    return millis() - s_last_heartbeat_ms[static_cast<size_t>(id)];
}

uint8_t CheckHeartbeats() {
    uint8_t faulted = 0;
    for (size_t i = 0; i < static_cast<size_t>(TaskId::COUNT); ++i) {
        if (MsSinceLastHeartbeat(static_cast<TaskId>(i)) > WATCHDOG_TIMEOUT_MS) {
            faulted |= (1 << i);
        }
    }
    return faulted;
}

} // namespace Watchdog
