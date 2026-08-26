#include <Arduino.h>
#include <ESP32Servo.h>
#include "gripper_task.h"
#include "config.h"
#include "queues.h"
#include "watchdog.h"

namespace {

// TODO: calibrar estos ángulos con el gripper real.
constexpr int kClawOpenDeg   = 30;
constexpr int kClawClosedDeg = 120;
constexpr int kLiftUpDeg     = 160;
constexpr int kLiftDownDeg   = 20;

void GripperTaskFn(void *) {
    Servo claw_servo;
    Servo lift_servo;
    claw_servo.attach(Pins::SERVO_GRIPPER_CLAW);
    lift_servo.attach(Pins::SERVO_GRIPPER_LIFT);
    claw_servo.write(kClawOpenDeg);
    lift_servo.write(kLiftUpDeg);

    const TickType_t period = pdMS_TO_TICKS(TaskPeriodMs::GRIPPER);
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        GripperCommand cmd;
        if (xQueueReceive(g_gripperCmdQueue, &cmd, 0) == pdTRUE) {
            // Cada acción es independiente y se aplica de inmediato; si el
            // servo físico no responde (mecanismo trabado, etc.) esta tarea
            // simplemente sigue intentando en el próximo comando — no
            // bloquea a MotorTask ni a los sensores.
            switch (cmd.action) {
                case GripperAction::OPEN:  claw_servo.write(kClawOpenDeg);   break;
                case GripperAction::CLOSE: claw_servo.write(kClawClosedDeg); break;
                case GripperAction::RAISE: lift_servo.write(kLiftUpDeg);     break;
                case GripperAction::LOWER: lift_servo.write(kLiftDownDeg);   break;
            }
        }

        Watchdog::ReportHeartbeat(TaskId::GRIPPER_CONTROL);
        vTaskDelayUntil(&last_wake, period);
    }
}

} // namespace

void GripperTask_Start() {
    xTaskCreatePinnedToCore(
        GripperTaskFn, "GripperTask",
        TaskStack::GRIPPER_CONTROL, nullptr,
        TaskPriority::GRIPPER_CONTROL, nullptr,
        /*core=*/1);
}
