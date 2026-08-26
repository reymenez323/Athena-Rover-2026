#include <Arduino.h>
#include "supervisor_task.h"
#include "config.h"
#include "queues.h"
#include "watchdog.h"

// -----------------------------------------------------------------------
// Estrategia de tolerancia a fallos de este proyecto (resumen — ver detalle
// completo en docs/arquitectura-firmware-esp32.md):
//
// 1. Cada tarea es dueña de su propio hardware y su propia cola: un fallo de
//    lectura del sensor de color, por ejemplo, nunca puede bloquear la cola
//    de motores ni la del gripper.
// 2. Cada tarea reporta un heartbeat en cada iteración. SupervisorTask corre
//    con alta prioridad y detecta si alguna dejó de reportar (colgada en un
//    I2C, un servo que no responde, etc.).
// 3. Cuando detecta una tarea colgada, SupervisorTask NO reinicia el ESP32
//    completo: registra el fallo y lo manda a la RPi por telemetría
//    (HealthReport) para que la capa de decisión sepa que, por ejemplo, ya
//    no hay lecturas de color confiables y actúe en consecuencia (frenar,
//    pedir ayuda, etc.), mientras el resto de tareas (motores, gripper,
//    comunicación) siguen funcionando con normalidad.
// 4. Límite conocido: el ESP32 no tiene aislamiento de memoria por tarea
//    (no hay MMU/procesos como en un SO de escritorio), así que esto
//    protege contra tareas "colgadas" (bucles infinitos, esperas sin
//    timeout, periféricos que no responden) pero NO contra corrupción de
//    memoria (un overflow de stack severo sí puede afectar a otras tareas).
//    Por eso cada TaskStack en config.h debe revisarse con
//    uxTaskGetStackHighWaterMark() durante las pruebas.
// -----------------------------------------------------------------------

namespace {

void SupervisorTaskFn(void *) {
    Watchdog::Init();

    const TickType_t period = pdMS_TO_TICKS(TaskPeriodMs::SUPERVISOR);
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        uint8_t faulted = Watchdog::CheckHeartbeats();

        if (faulted != 0) {
            HealthReport report;
            report.timestamp_ms = millis();
            report.faulted_tasks_bitmask = faulted;
            // No bloquear si la cola está llena; la RPi recibirá el próximo reporte.
            xQueueOverwrite(g_healthTelemetryQueue, &report);

#if CORE_DEBUG_LEVEL >= 3
            Serial.printf("[Supervisor] Tareas con fallo (bitmask=0x%02X)\n", faulted);
#endif
        }

        // SupervisorTask también se reporta a sí misma para que quede
        // registro de que el propio supervisor sigue vivo (visible en logs).
        vTaskDelayUntil(&last_wake, period);
    }
}

} // namespace

void SupervisorTask_Start() {
    xTaskCreatePinnedToCore(
        SupervisorTaskFn, "SupervisorTask",
        TaskStack::SUPERVISOR, nullptr,
        TaskPriority::SUPERVISOR, nullptr,
        /*core=*/0);
}
