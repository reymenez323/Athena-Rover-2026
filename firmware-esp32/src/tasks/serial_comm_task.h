#pragma once
// serial_comm_task.h — Puente entre el enlace serial con la Raspberry Pi y
// las colas internas de FreeRTOS. Es la ÚNICA tarea que toca el UART hacia
// la RPi, para no repartir el acceso al puerto entre varias tareas.
void SerialCommTask_Start();
