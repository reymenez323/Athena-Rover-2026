#include <Arduino.h>
#include <Wire.h>
#include "color_sensor_task.h"
#include "config.h"
#include "queues.h"
#include "watchdog.h"

namespace {

// TODO: reemplazar por la librería real del sensor elegido (p.ej. Adafruit_TCS34725).
// Esta función está aislada a propósito: si el sensor falla o no responde,
// devuelve `false` y el resto del firmware sigue funcionando con el último
// dato válido — un sensor de color roto no debe detener al robot.
bool ReadColorSensor(bool is_front, ColorReading::Label &out_label) {
    // Placeholder: TODO integrar lectura real por I2C.
    out_label = ColorReading::Label::UNKNOWN;
    return false;
}

void ColorSensorTaskFn(void *) {
    Wire.begin(Pins::I2C_SDA, Pins::I2C_SCL);

    const TickType_t period = pdMS_TO_TICKS(TaskPeriodMs::COLOR_SENSOR);
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        ColorReading reading;
        reading.timestamp_ms = millis();
        reading.front_valid = ReadColorSensor(true,  reading.front_label);
        reading.back_valid  = ReadColorSensor(false, reading.back_label);

        // No bloquear si SerialCommTask está saturada: si la cola está
        // llena, se descarta la lectura más vieja y seguimos — preferimos
        // telemetría fresca a telemetría completa.
        if (xQueueSend(g_colorTelemetryQueue, &reading, 0) != pdTRUE) {
            ColorReading discard;
            xQueueReceive(g_colorTelemetryQueue, &discard, 0);
            xQueueSend(g_colorTelemetryQueue, &reading, 0);
        }

        Watchdog::ReportHeartbeat(TaskId::COLOR_SENSOR);
        vTaskDelayUntil(&last_wake, period);
    }
}

} // namespace

void ColorSensorTask_Start() {
    xTaskCreatePinnedToCore(
        ColorSensorTaskFn, "ColorSensorTask",
        TaskStack::COLOR_SENSOR, nullptr,
        TaskPriority::COLOR_SENSOR, nullptr,
        /*core=*/0);
}
