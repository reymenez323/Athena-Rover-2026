#include <Arduino.h>
#include "motor_task.h"
#include "config.h"
#include "queues.h"
#include "watchdog.h"

namespace {

struct MotorPins { uint8_t in1, in2, pwm; };

const MotorPins kFrontLeft  = {Pins::MOTOR_FL_IN1, Pins::MOTOR_FL_IN2, Pins::MOTOR_FL_PWM};
const MotorPins kFrontRight = {Pins::MOTOR_FR_IN1, Pins::MOTOR_FR_IN2, Pins::MOTOR_FR_PWM};
const MotorPins kRearLeft   = {Pins::MOTOR_RL_IN1, Pins::MOTOR_RL_IN2, Pins::MOTOR_RL_PWM};
const MotorPins kRearRight  = {Pins::MOTOR_RR_IN1, Pins::MOTOR_RR_IN2, Pins::MOTOR_RR_PWM};

void SetupMotorPins(const MotorPins &m) {
    pinMode(m.in1, OUTPUT);
    pinMode(m.in2, OUTPUT);
    pinMode(m.pwm, OUTPUT);
}

// speed: -100..100. Signo = dirección, magnitud = duty cycle.
void ApplyMotor(const MotorPins &m, int8_t speed) {
    speed = constrain(speed, (int8_t)-100, (int8_t)100);
    digitalWrite(m.in1, speed >= 0 ? HIGH : LOW);
    digitalWrite(m.in2, speed >= 0 ? LOW  : HIGH);
    uint8_t duty = map(abs(speed), 0, 100, 0, 255);
    analogWrite(m.pwm, duty); // TODO: migrar a ledcWrite (LEDC) si analogWrite no está disponible en el core usado
}

void StopAllMotors() {
    ApplyMotor(kFrontLeft, 0);
    ApplyMotor(kFrontRight, 0);
    ApplyMotor(kRearLeft, 0);
    ApplyMotor(kRearRight, 0);
}

void MotorTaskFn(void *) {
    SetupMotorPins(kFrontLeft);
    SetupMotorPins(kFrontRight);
    SetupMotorPins(kRearLeft);
    SetupMotorPins(kRearRight);
    StopAllMotors();

    MotorCommand current{}; // por defecto: STOP
    uint32_t last_valid_cmd_ms = millis();

    const TickType_t period = pdMS_TO_TICKS(TaskPeriodMs::MOTOR_CONTROL);
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        // Nunca bloquear indefinidamente: si no hay comando nuevo, seguimos
        // aplicando el último válido (o el failsafe) sin depender de que
        // SerialCommTask esté vivo en este instante.
        MotorCommand incoming;
        if (xQueueReceive(g_motorCmdQueue, &incoming, 0) == pdTRUE) {
            current = incoming;
            last_valid_cmd_ms = millis();
        }

        // Failsafe de comunicación: propio de esta tarea, no depende de que
        // otra tarea lo detecte por nosotros.
        bool comms_stale = (millis() - last_valid_cmd_ms) > COMMS_FAILSAFE_TIMEOUT_MS;

        if (comms_stale || current.type == MotorCmdType::STOP) {
            StopAllMotors();
        } else {
            ApplyMotor(kFrontLeft,  current.left_speed);
            ApplyMotor(kRearLeft,   current.left_speed);
            ApplyMotor(kFrontRight, current.right_speed);
            ApplyMotor(kRearRight,  current.right_speed);
        }

        Watchdog::ReportHeartbeat(TaskId::MOTOR_CONTROL);
        vTaskDelayUntil(&last_wake, period);
    }
}

} // namespace

void MotorTask_Start() {
    xTaskCreatePinnedToCore(
        MotorTaskFn, "MotorTask",
        TaskStack::MOTOR_CONTROL, nullptr,
        TaskPriority::MOTOR_CONTROL, nullptr,
        /*core=*/1);
}
