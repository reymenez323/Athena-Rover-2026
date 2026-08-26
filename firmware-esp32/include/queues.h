#pragma once
// queues.h — Colas FreeRTOS globales, una por subsistema.
//
// Cada cola tiene UN productor y UN consumidor lógico, lo que evita
// contención y hace explícito quién depende de quién:
//
//   RPi --(serial)--> SerialCommTask --> g_motorCmdQueue (len 1, overwrite) --> MotorTask
//                                    --> g_gripperCmdQueue (len 4, fifo)    --> GripperTask
//                                    --> g_ledCmdQueue (len 1, overwrite)   --> LedTask
//
//   ColorSensorTask  --> g_colorTelemetryQueue (len 4)       --> SerialCommTask --(serial)--> RPi
//   ReflectanceTask  --> g_reflectanceTelemetryQueue (len 4) --> SerialCommTask --(serial)--> RPi
//   SupervisorTask   --> g_healthTelemetryQueue (len 1, overwrite) --> SerialCommTask --(serial)--> RPi
//
// Las colas "overwrite" (largo 1) son para datos donde solo importa el valor
// más reciente (comando de motores, LED, reporte de salud). Las colas FIFO
// (largo 4) son para eventos que deben procesarse todos, en orden.
//
// Todas se crean en main.cpp ANTES de lanzar las tareas, y se consultan
// siempre con timeout (nunca portMAX_DELAY en un loop de tarea) para que un
// productor lento no cuelgue indefinidamente al consumidor, ni viceversa.

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "shared_types.h"

extern QueueHandle_t g_motorCmdQueue;         // MotorCommand,      largo 1 (overwrite)
extern QueueHandle_t g_gripperCmdQueue;       // GripperCommand,    largo 4 (fifo)
extern QueueHandle_t g_ledCmdQueue;           // LedCommand,        largo 1 (overwrite)

extern QueueHandle_t g_colorTelemetryQueue;       // ColorReading,       largo 4 (fifo)
extern QueueHandle_t g_reflectanceTelemetryQueue; // ReflectanceReading, largo 4 (fifo)
extern QueueHandle_t g_healthTelemetryQueue;      // HealthReport,       largo 1 (overwrite)

// Crea todas las colas. Debe llamarse una sola vez desde setup(), antes de
// crear ninguna tarea. Si alguna cola falla al crearse (memoria insuficiente),
// se hace un log claro por serial de depuración y se detiene el arranque:
// es preferible fallar rápido en el banco de pruebas que arrancar a medias.
void InitQueues();
