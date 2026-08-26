# Arquitectura del firmware ESP32-S3 — FreeRTOS

## Objetivo

El ESP32-S3 se encarga de accionar y leer todos los actuadores/sensores del robot,
respondiendo a comandos de la Raspberry Pi por puerto serial. El requisito de diseño
explícito del equipo es: **cada subsistema corre en su propia tarea de FreeRTOS,
independiente de las demás, y un fallo en una tarea no debe tumbar ni bloquear a las otras.**

## Tareas

| Tarea             | Responsabilidad                                          | Prioridad | Core | Periodo   |
|-------------------|-----------------------------------------------------------|:---------:|:----:|-----------|
| SupervisorTask    | Vigila heartbeats de todas las tareas, reporta fallos      | 5 (alta)  | 0    | 200 ms    |
| SerialCommTask    | Único punto de contacto con el UART de la RPi              | 4         | 1    | ~5 ms     |
| MotorTask         | Controla los 4 motores de tracción                         | 4         | 1    | 20 ms     |
| GripperTask       | Controla los 2 servos del gripper (pinza y elevación)       | 3         | 1    | 50 ms     |
| ReflectanceTask   | Lee los 2 sensores de reflectancia (izq./der.)              | 3         | 0    | 20 ms     |
| ColorSensorTask   | Lee los sensores de color delantero y trasero               | 2         | 0    | 100 ms    |
| LedTask           | Controla el LED indicador de equipo (rojo/azul)             | 1 (baja)  | 0    | 250 ms    |

Las tareas de control físico crítico (motores, comunicación) se fijan al núcleo 1;
las de sensado y estado se fijan al núcleo 0, para que un sensor lento no le quite
tiempo de CPU al lazo de control de motores.

## Comunicación entre tareas: colas, no estado global

No existe un "estado global del robot" compartido por mutex. En vez de eso, cada
tarea es dueña de sus propios datos y los publica a través de colas de FreeRTOS
(`include/queues.h`), cada una con un productor y un consumidor claros:

```
RPi ⇄ SerialCommTask ⇄ [motorCmdQueue]       ⇄ MotorTask
                      ⇄ [gripperCmdQueue]     ⇄ GripperTask
                      ⇄ [ledCmdQueue]         ⇄ LedTask
                      ⇄ [colorTelemetryQueue]       ⇐ ColorSensorTask
                      ⇄ [reflectanceTelemetryQueue] ⇐ ReflectanceTask
                      ⇄ [healthTelemetryQueue]      ⇐ SupervisorTask
```

Reglas seguidas en todo el código:

- **Ninguna tarea bloquea indefinidamente** (`portMAX_DELAY`) en una cola dentro de
  su loop principal. Todas las lecturas/escrituras de cola usan timeout 0 o corto,
  así que un productor o consumidor lento nunca cuelga a otra tarea.
- Las colas de **comando** (motor, LED) son de largo 1 y usan `xQueueOverwrite`:
  solo importa el comando más reciente, nunca se acumulan órdenes viejas.
- La cola de **gripper** es FIFO (largo 4): las acciones (abrir/cerrar/subir/bajar)
  deben ejecutarse en el orden en que llegaron.
- Las colas de **telemetría** son FIFO cortas; si se llenan porque `SerialCommTask`
  no alcanza a vaciarlas, se descarta el dato más viejo — se prefiere telemetría
  fresca a telemetría completa.

## Tolerancia a fallos

### 1. Falla de hardware/sensor dentro de una tarea

Cada tarea maneja sus propios errores de I/O (I2C que no responde, servo que no se
mueve, etc.) de forma local: registra el problema y sigue con el último dato válido
o un valor seguro por defecto, en vez de propagar una excepción o quedarse esperando
sin límite de tiempo. Ejemplo: si el sensor de color falla, `ColorSensorTask` sigue
corriendo y reportando `front_valid = false` — no detiene el robot ni bloquea a
`MotorTask`.

### 2. Pérdida de comunicación con la Raspberry Pi

`MotorTask` implementa su **propio** failsafe: si no recibe un comando válido de
`SerialCommTask` en `COMMS_FAILSAFE_TIMEOUT_MS` (500 ms), detiene los motores por su
cuenta. No depende de que otra tarea se lo indique.

### 3. Tarea colgada (bucle infinito, espera sin timeout, periférico que no responde)

Cada tarea reporta un "heartbeat" en cada iteración de su loop (`Watchdog::ReportHeartbeat`,
ver `src/watchdog.*`). `SupervisorTask` corre con la prioridad más alta del sistema y
revisa cada 200 ms si alguna tarea dejó de reportar dentro de `WATCHDOG_TIMEOUT_MS`
(1 s). Si detecta una tarea colgada:

1. Registra el fallo (bitmask de qué tarea(s) fallaron).
2. Lo envía a la Raspberry Pi vía `HealthReport`, para que la capa de decisión (que
   corre en la RPi) sepa qué subsistema dejó de responder y actúe en consecuencia.
3. El resto de las tareas **sigue funcionando con normalidad** — el supervisor no
   reinicia el ESP32 completo por el fallo de una sola tarea.

### Límite conocido (honestidad técnica)

El ESP32-S3 no tiene aislamiento de memoria por proceso (no hay MMU como en un
sistema operativo de escritorio): todas las tareas comparten el mismo espacio de
memoria. Este diseño protege contra:

- Tareas que se cuelgan esperando un periférico o en un bucle sin salida.
- Errores de lectura de sensores o falta de respuesta de un actuador.
- Pérdida de comunicación con la Raspberry Pi.

Pero **no** protege completamente contra corrupción de memoria (por ejemplo, un
desbordamiento severo de stack en una tarea sí podría afectar a otra). Por eso:

- Cada `TaskStack::*` en `config.h` es un punto de partida conservador; hay que
  medirlo con `uxTaskGetStackHighWaterMark()` durante las pruebas en banco y ajustar.
- Se evita el uso de memoria dinámica (`new`/`malloc`) dentro de los loops de las
  tareas, una vez pasada la inicialización.

## Próximos pasos sugeridos

1. Confirmar el modelo exacto de sensor de color y de reflectancia, e implementar
   las lecturas reales en `color_sensor_task.cpp` / `reflectance_task.cpp` (hoy son
   placeholders marcados con `TODO`).
2. Confirmar pines definitivos en `include/config.h` una vez esté el esquemático
   eléctrico en `hardware/`.
3. Definir en la Raspberry Pi el lado equivalente del protocolo serial (mismo
   formato de paquete `[0xAA][TYPE][LEN][PAYLOAD][CHECKSUM]`) para poder probar el
   enlace de punta a punta.
4. Probar cada tarea de forma aislada (banco de pruebas) antes de integrarlas todas.
