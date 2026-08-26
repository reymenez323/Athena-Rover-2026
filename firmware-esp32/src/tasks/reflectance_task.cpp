#include <Arduino.h>
#include "reflectance_task.h"
#include "config.h"
#include "queues.h"
#include "watchdog.h"

namespace {

// TODO: calibrar este umbral en la pista real (varía con luz ambiente y sensor elegido).
constexpr uint16_t kLineThreshold = 2048;

void ReflectanceTaskFn(void *) {
    pinMode(Pins::REFLECTANCE_LEFT, INPUT);
    pinMode(Pins::REFLECTANCE_RIGHT, INPUT);

    const TickType_t period = pdMS_TO_TICKS(TaskPeriodMs::REFLECTANCE);
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        ReflectanceReading reading;
        reading.timestamp_ms = millis();
        reading.left_raw  = analogRead(Pins::REFLECTANCE_LEFT);
        reading.right_raw = analogRead(Pins::REFLECTANCE_RIGHT);
        reading.left_on_line  = reading.left_raw  > kLineThreshold;
        reading.right_on_line = reading.right_raw > kLineThreshold;

        if (xQueueSend(g_reflectanceTelemetryQueue, &reading, 0) != pdTRUE) {
            ReflectanceReading discard;
            xQueueReceive(g_reflectanceTelemetryQueue, &discard, 0);
            xQueueSend(g_reflectanceTelemetryQueue, &reading, 0);
        }

        Watchdog::ReportHeartbeat(TaskId::REFLECTANCE);
        vTaskDelayUntil(&last_wake, period);
    }
}

} // namespace

void ReflectanceTask_Start() {
    xTaskCreatePinnedToCore(
        ReflectanceTaskFn, "ReflectanceTask",
        TaskStack::REFLECTANCE, nullptr,
        TaskPriority::REFLECTANCE, nullptr,
        /*core=*/0);
}
