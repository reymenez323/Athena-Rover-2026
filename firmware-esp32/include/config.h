#pragma once
// config.h — Pines, prioridades de tareas y constantes globales del firmware.
// Ajustar los pines TODO según el diagrama de conexiones final (ver hardware/).

#include <Arduino.h>

// ---------------------------------------------------------------------------
// Pines — TODO: confirmar con el esquemático final antes de fabricar el PCB.
// ---------------------------------------------------------------------------
namespace Pins {
    // Motores (ej. puente H tipo TB6612FNG / L298N, 2 canales x 2 motores en paralelo por lado)
    constexpr uint8_t MOTOR_FL_IN1 = 4,  MOTOR_FL_IN2 = 5,  MOTOR_FL_PWM = 6;
    constexpr uint8_t MOTOR_FR_IN1 = 7,  MOTOR_FR_IN2 = 8,  MOTOR_FR_PWM = 9;
    constexpr uint8_t MOTOR_RL_IN1 = 10, MOTOR_RL_IN2 = 11, MOTOR_RL_PWM = 12;
    constexpr uint8_t MOTOR_RR_IN1 = 13, MOTOR_RR_IN2 = 14, MOTOR_RR_PWM = 15;

    // Servos del gripper
    constexpr uint8_t SERVO_GRIPPER_CLAW = 16;   // abre/cierra la pinza
    constexpr uint8_t SERVO_GRIPPER_LIFT = 17;   // sube/baja el gripper

    // Sensores de color (I2C, ej. TCS34725) — front y back comparten el bus I2C
    // pero cada uno tiene su propio pin de habilitación/dirección si aplica.
    constexpr uint8_t I2C_SDA = 8;
    constexpr uint8_t I2C_SCL = 9;
    constexpr uint8_t COLOR_SENSOR_FRONT_XSHUT = 18; // si el sensor lo requiere
    constexpr uint8_t COLOR_SENSOR_BACK_XSHUT  = 21;

    // Sensores de reflectancia (analógicos o digitales, izquierda/derecha)
    constexpr uint8_t REFLECTANCE_LEFT  = A0;
    constexpr uint8_t REFLECTANCE_RIGHT = A1;

    // LED indicador de equipo (rojo/azul) — usar LED RGB o 2 LEDs simples
    constexpr uint8_t LED_TEAM_RED  = 38;
    constexpr uint8_t LED_TEAM_BLUE = 39;

    // UART hacia la Raspberry Pi (usar UART1 o UART2 del ESP32-S3, no el de programación/USB)
    constexpr uint8_t RPI_UART_RX = 44;
    constexpr uint8_t RPI_UART_TX = 43;
}

// ---------------------------------------------------------------------------
// Comunicación serial con la Raspberry Pi
// ---------------------------------------------------------------------------
namespace SerialLink {
    constexpr uint32_t BAUD_RATE = 115200;
    constexpr uint32_t RX_TIMEOUT_MS = 20;
}

// ---------------------------------------------------------------------------
// Prioridades de tareas FreeRTOS (mayor número = mayor prioridad).
// Se mantienen deliberadamente bajas y separadas para que el scheduler
// tenga margen; nada corre en configMAX_PRIORITIES - 1.
// ---------------------------------------------------------------------------
namespace TaskPriority {
    constexpr UBaseType_t SUPERVISOR      = 5; // debe seguir corriendo pase lo que pase
    constexpr UBaseType_t SERIAL_COMM     = 4; // enlace con RPi, no debe acumular retraso
    constexpr UBaseType_t MOTOR_CONTROL   = 4; // crítico para seguridad (parada de emergencia)
    constexpr UBaseType_t GRIPPER_CONTROL = 3;
    constexpr UBaseType_t REFLECTANCE     = 3; // realimentación rápida de línea
    constexpr UBaseType_t COLOR_SENSOR    = 2;
    constexpr UBaseType_t LED_STATUS      = 1;
}

// Tamaños de stack (en palabras de 4 bytes en ESP32, no bytes) — punto de partida,
// ajustar con uxTaskGetStackHighWaterMark() durante pruebas.
namespace TaskStack {
    constexpr uint32_t SUPERVISOR      = 3072;
    constexpr uint32_t SERIAL_COMM     = 4096;
    constexpr uint32_t MOTOR_CONTROL   = 3072;
    constexpr uint32_t GRIPPER_CONTROL = 2048;
    constexpr uint32_t REFLECTANCE     = 2048;
    constexpr uint32_t COLOR_SENSOR    = 3072;
    constexpr uint32_t LED_STATUS      = 1536;
}

// Periodos de ejecución de cada tarea (ms)
namespace TaskPeriodMs {
    constexpr uint32_t SUPERVISOR    = 200;
    constexpr uint32_t MOTOR_CONTROL = 20;   // 50 Hz
    constexpr uint32_t GRIPPER       = 50;
    constexpr uint32_t REFLECTANCE   = 20;   // 50 Hz, seguimiento de línea
    constexpr uint32_t COLOR_SENSOR  = 100;  // 10 Hz, suficiente para detectar zonas
    constexpr uint32_t LED_STATUS    = 250;
}

// Tiempo máximo sin heartbeat antes de que el supervisor considere una tarea "colgada".
constexpr uint32_t WATCHDOG_TIMEOUT_MS = 1000;

// Si SerialCommTask no recibe un comando válido de la RPi en este tiempo,
// MotorTask entra en modo de parada de seguridad (failsafe) de forma independiente.
constexpr uint32_t COMMS_FAILSAFE_TIMEOUT_MS = 500;

// Identificadores de tarea, usados como índice en el arreglo de heartbeats del watchdog.
enum class TaskId : uint8_t {
    SERIAL_COMM = 0,
    MOTOR_CONTROL,
    GRIPPER_CONTROL,
    COLOR_SENSOR,
    REFLECTANCE,
    LED_STATUS,
    COUNT // debe ser siempre el último
};
