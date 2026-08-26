#pragma once
// shared_types.h — Structs que viajan por las colas FreeRTOS entre tareas,
// y por el protocolo serial con la Raspberry Pi.
//
// Regla de diseño: cada struct pertenece a UNA sola cola / dueño. Evitamos un
// "estado global" gigante compartido por mutex; en vez de eso cada tarea posee
// sus propios datos y los publica a quien le interese vía cola. Esto reduce el
// acoplamiento y evita que un deadlock en un subsistema bloquee a los demás.

#include <cstdint>

enum class TeamColor : uint8_t { NONE = 0, RED = 1, BLUE = 2 };

// -------------------- Comandos: Raspberry Pi -> ESP32 --------------------

enum class MotorCmdType : uint8_t { STOP = 0, DRIVE = 1 };

struct MotorCommand {
    MotorCmdType type = MotorCmdType::STOP;
    int8_t left_speed  = 0; // -100..100 (%), lado izquierdo (motores FL+RL)
    int8_t right_speed = 0; // -100..100 (%), lado derecho (motores FR+RR)
};

enum class GripperAction : uint8_t { OPEN = 0, CLOSE = 1, RAISE = 2, LOWER = 3 };

struct GripperCommand {
    GripperAction action = GripperAction::OPEN;
};

struct LedCommand {
    TeamColor team = TeamColor::NONE;
};

// -------------------- Telemetría: ESP32 -> Raspberry Pi --------------------

struct ColorReading {
    uint32_t timestamp_ms = 0;
    bool front_valid = false;
    bool back_valid  = false;
    // Color detectado clasificado en categorías conocidas de la pista.
    enum class Label : uint8_t { UNKNOWN = 0, BLACK, YELLOW, RED, BLUE, FLOOR } front_label = Label::UNKNOWN,
                                                                                  back_label  = Label::UNKNOWN;
};

struct ReflectanceReading {
    uint32_t timestamp_ms = 0;
    uint16_t left_raw  = 0;
    uint16_t right_raw = 0;
    bool left_on_line  = false;
    bool right_on_line = false;
};

// Estado de salud reportado por el supervisor — permite que la RPi sepa si algún
// subsistema falló, sin que eso tumbe la comunicación general.
struct HealthReport {
    uint32_t timestamp_ms = 0;
    uint8_t faulted_tasks_bitmask = 0; // bit i = TaskId i encontrado colgado desde el último reporte
};

// Envelope genérico que SerialCommTask arma para mandar telemetría hacia la RPi.
// (En vez de una unión insegura, se serializa cada tipo con su propio tag —
// ver serial_comm_task.cpp para el formato exacto sobre el alambre.)
enum class TelemetryType : uint8_t { COLOR = 0, REFLECTANCE = 1, HEALTH = 2 };
