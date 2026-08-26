// main.cpp — Punto de entrada del firmware ESP32-S3 (Athena Rover 2026).
//
// Arquitectura: cada subsistema del robot vive en su propia tarea FreeRTOS,
// independiente de las demás, comunicadas solo por colas explícitas (ver
// include/queues.h) y vigiladas por un supervisor con heartbeat (ver
// src/watchdog.*). El diseño completo está documentado en
// docs/arquitectura-firmware-esp32.md — léanlo antes de tocar este archivo.
//
// setup() SOLO crea las colas y lanza las tareas. Ninguna lógica de negocio
// vive aquí ni en loop().

#include <Arduino.h>
#include "config.h"
#include "queues.h"

#include "tasks/motor_task.h"
#include "tasks/gripper_task.h"
#include "tasks/color_sensor_task.h"
#include "tasks/reflectance_task.h"
#include "tasks/led_task.h"
#include "tasks/serial_comm_task.h"
#include "tasks/supervisor_task.h"

QueueHandle_t g_motorCmdQueue = nullptr;
QueueHandle_t g_gripperCmdQueue = nullptr;
QueueHandle_t g_ledCmdQueue = nullptr;
QueueHandle_t g_colorTelemetryQueue = nullptr;
QueueHandle_t g_reflectanceTelemetryQueue = nullptr;
QueueHandle_t g_healthTelemetryQueue = nullptr;

void InitQueues() {
    g_motorCmdQueue   = xQueueCreate(1, sizeof(MotorCommand));
    g_gripperCmdQueue = xQueueCreate(4, sizeof(GripperCommand));
    g_ledCmdQueue     = xQueueCreate(1, sizeof(LedCommand));

    g_colorTelemetryQueue       = xQueueCreate(4, sizeof(ColorReading));
    g_reflectanceTelemetryQueue = xQueueCreate(4, sizeof(ReflectanceReading));
    g_healthTelemetryQueue      = xQueueCreate(1, sizeof(HealthReport));

    bool ok = g_motorCmdQueue && g_gripperCmdQueue && g_ledCmdQueue &&
              g_colorTelemetryQueue && g_reflectanceTelemetryQueue && g_healthTelemetryQueue;

    if (!ok) {
        Serial.println("[FATAL] No se pudieron crear las colas FreeRTOS. Deteniendo arranque.");
        while (true) { delay(1000); }
    }
}

void setup() {
    Serial.begin(115200); // puerto USB, solo para depuración local
    delay(200);
    Serial.println("Athena Rover 2026 — firmware ESP32-S3 iniciando...");

    InitQueues();

    // Orden de arranque: el supervisor primero, para que pueda detectar
    // desde el inicio si alguna tarea tarda demasiado en arrancar.
    SupervisorTask_Start();
    SerialCommTask_Start();
    MotorTask_Start();
    GripperTask_Start();
    ColorSensorTask_Start();
    ReflectanceTask_Start();
    LedTask_Start();

    Serial.println("Todas las tareas fueron lanzadas.");
}

void loop() {
    // Todo el trabajo ocurre en las tareas FreeRTOS; loop() se mantiene
    // deliberadamente vacío (la tarea de Arduino en sí corre con prioridad 1).
    vTaskDelay(pdMS_TO_TICKS(1000));
}
