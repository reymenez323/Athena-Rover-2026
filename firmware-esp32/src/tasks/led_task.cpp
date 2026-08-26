#include <Arduino.h>
#include "led_task.h"
#include "config.h"
#include "queues.h"
#include "watchdog.h"

namespace {

void ApplyTeamColor(TeamColor team) {
    digitalWrite(Pins::LED_TEAM_RED,  team == TeamColor::RED);
    digitalWrite(Pins::LED_TEAM_BLUE, team == TeamColor::BLUE);
}

void LedTaskFn(void *) {
    pinMode(Pins::LED_TEAM_RED, OUTPUT);
    pinMode(Pins::LED_TEAM_BLUE, OUTPUT);

    TeamColor current_team = TeamColor::NONE;
    const TickType_t period = pdMS_TO_TICKS(TaskPeriodMs::LED_STATUS);
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        LedCommand cmd;
        if (xQueueReceive(g_ledCmdQueue, &cmd, 0) == pdTRUE) {
            current_team = cmd.team;
        }
        ApplyTeamColor(current_team);

        Watchdog::ReportHeartbeat(TaskId::LED_STATUS);
        vTaskDelayUntil(&last_wake, period);
    }
}

} // namespace

void LedTask_Start() {
    xTaskCreatePinnedToCore(
        LedTaskFn, "LedTask",
        TaskStack::LED_STATUS, nullptr,
        TaskPriority::LED_STATUS, nullptr,
        /*core=*/0);
}
