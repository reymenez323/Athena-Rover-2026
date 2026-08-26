# Firmware ESP32-S3 — Athena Rover 2026

Firmware basado en **FreeRTOS** (vía framework Arduino-ESP32) que controla todos los actuadores
y sensores del robot, y se comunica por puerto serial con la Raspberry Pi (que lleva la
inteligencia de alto nivel: visión, decisiones de misión).

Ver [`../docs/arquitectura-firmware-esp32.md`](../docs/arquitectura-firmware-esp32.md) para el
diseño completo de tareas, colas y estrategia de tolerancia a fallos.

## Estructura

```
firmware-esp32/
├── platformio.ini
├── include/
│   ├── config.h          # Pines, prioridades de tareas, tamaños de stack, timeouts
│   └── shared_types.h    # Structs compartidos entre tareas (comandos, telemetría, estado)
└── src/
    ├── main.cpp           # Punto de entrada: crea colas/mutex y lanza todas las tareas
    ├── watchdog.{h,cpp}   # Registro de "heartbeat" por tarea, usado por el supervisor
    └── tasks/
        ├── motor_task.*        # Control de los 4 motores de tracción
        ├── gripper_task.*      # Control de los 2 servos (pinza + subir/bajar)
        ├── color_sensor_task.* # Sensores de color delantero y trasero
        ├── reflectance_task.*  # Sensores de reflectancia izquierdo/derecho
        ├── serial_comm_task.*  # Protocolo serial con la Raspberry Pi
        ├── led_task.*          # LED indicador de equipo (rojo/azul)
        └── supervisor_task.*   # Vigila el heartbeat de cada tarea y reacciona a fallos
```

## Compilar / subir

```bash
pio run                # compilar
pio run -t upload      # subir al ESP32-S3
pio device monitor      # ver logs por serial
```
