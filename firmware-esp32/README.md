# Firmware ESP32-S3 — Athena Rover 2026

Todo el firmware vive en **un solo archivo**: `src/main.cpp`. Cada subsistema
del robot corre en su propia tarea de FreeRTOS, independiente de las demás.

**Dependencias externas: ninguna.** Solo el framework Arduino-ESP32 (que ya trae
FreeRTOS) y su librería `Wire`. Los drivers del PCA9685 y del TCS34725 están
escritos dentro del mismo archivo: son un puñado de registros I2C cada uno, y
escribirlos nos da control total sobre los tiempos y la ganancia — que es justo
lo que hay que ajustar al calibrar.

## Hardware

| Componente | Cantidad | Para qué |
|------------|:--------:|----------|
| L298N | 2 | 4 motores de tracción (cada driver mueve 2) |
| PCA9685 | 1 | **1** servo del gripper (la pinza: agarra o suelta), por I2C |
| TCS34725 | 2 | Color del piso, delantero y trasero |
| QTRX-HD-01A | 2 | Reflectancia delantera, izquierda y derecha |
| LED RGB | 1 | Identificación de equipo (lo exige el reglamento) **y** señalización de la bandera contraria |
| VL53L1X | 1 | ToF delante del gripper: distancia real a la bandera cilíndrica |

**Las conexiones completas están en [`../hardware/conexiones-esp32-s3.md`](../hardware/conexiones-esp32-s3.md).**
Léelo antes de cablear: incluye los cinco errores que queman hardware.

## Compilar y subir

```bash
pio run              # compilar
pio run -t upload    # subir al ESP32-S3
pio device monitor   # ver los logs
```

## Las tareas

| Tarea | Prioridad | Núcleo | Periodo | Responsabilidad |
|-------|:---------:|:------:|:-------:|-----------------|
| Supervisor | 6 | 0 | 200 ms | Vigila los heartbeats de las demás |
| Motors | 5 | 1 | 20 ms | Los 4 motores + failsafe propio |
| SerialComm | 4 | 1 | 5 ms | Único punto de contacto con la Raspberry Pi |
| Gripper | 3 | 1 | 50 ms | El servo de la pinza, vía PCA9685 |
| Reflectance | 3 | 0 | 20 ms | Los 2 QTRX: borde negro vs. fondo gris |
| ColorSensors | 2 | 0 | 100 ms | Los 2 TCS34725: zonas de color del piso |
| TofSensor | 2 | 0 | 50 ms | El VL53L1X: distancia a la bandera |
| TeamLed | 1 | 0 | 250 ms | LED de equipo + señal de bandera a la vista |

Las tareas de control crítico van fijadas al **núcleo 1**; las de sensado y
estado al **núcleo 0**. Así un sensor lento no le quita tiempo de CPU al lazo
de control de los motores.

## Aislamiento entre tareas

No existe un "estado global del robot". Cada tarea es dueña de su hardware y de
sus datos, y los publica por colas explícitas de FreeRTOS.

```
RPi ──USB──> SerialTask ──> motorCmdQueue      (1, overwrite) ──> MotorTask
                        ──> gripperCmdQueue    (4, FIFO)      ──> GripperTask
                        ──> ledCmdQueue        (1, overwrite) ──> LedTask
                        ──> flagSignalCmdQueue (1, overwrite) ──> LedTask

ColorSensorTask ──> colorQueue   (4, FIFO)      ──┐
ReflectanceTask ──> reflectQueue (4, FIFO)      ──┼─> SerialTask ──USB──> RPi
SupervisorTask  ──> healthQueue  (1, overwrite) ──┘
```

Las colas *overwrite* (largo 1) son para datos donde solo importa el valor más
reciente: un comando de motores viejo no debe ejecutarse nunca. Las FIFO son
para eventos que deben procesarse todos y en orden (el gripper tiene que abrir
**antes** de bajar, no al revés).

**Regla que se respeta en todo el archivo:** ninguna tarea usa `portMAX_DELAY`
dentro de su bucle. Todas las operaciones de cola llevan timeout 0, así un
productor lento nunca cuelga al consumidor.

## Tolerancia a fallos

1. Cada tarea maneja sus propios errores de I2C. Los drivers devuelven `bool`
   y las tareas **reintentan en segundo plano** en vez de esperar a un chip que
   no contesta.
2. `Wire.setTimeOut(25)`: si un chip I2C se cuelga tirando SDA a masa, la
   transacción falla rápido en vez de congelar la tarea.
3. **`MotorTask` tiene failsafe propio**: si pasa 500 ms sin comando válido de
   la Raspberry Pi, frena por su cuenta. No depende de que nadie se lo diga.
4. Cada tarea marca un *heartbeat*. `SupervisorTask` detecta a la que dejó de
   marcar y lo reporta a la Pi — **sin reiniciar el ESP32**. Así la Pi puede
   dejar de confiar en ese sensor y seguir compitiendo.

### Límite conocido, dicho sin adornos

El ESP32-S3 **no tiene MMU**: todas las tareas comparten el mismo espacio de
memoria. Este diseño aísla tareas **colgadas** (bucles sin salida, periféricos
que no responden), **no memoria corrupta**: un desbordamiento de stack severo
en una tarea sí puede dañar a otra.

Por eso:
- Hay que medir los stacks reales con `uxTaskGetStackHighWaterMark()` en las
  pruebas de banco y ajustar los valores de `TaskStack`.
- Ninguna tarea reserva memoria dinámica dentro de su bucle.

## Protocolo con la Raspberry Pi

```
[0xAA][TYPE][LEN][PAYLOAD...][CHECKSUM]
```

El payload se arma y se lee **byte a byte**, en little-endian, nunca con
`memcpy` de un struct. Mandar structs crudos entre C++ y Python es una trampa
clásica: el compilador inserta padding invisible que no tiene por qué coincidir
con lo que asuma el otro lado.

La referencia legible del protocolo es
[`../docs/protocolo-serial.md`](../docs/protocolo-serial.md); el formato exacto
está en la sección `[3]` de `main.cpp` y el lado Python en
`raspberry-pi/src/athena/protocol.py`.

Hay tests que leen **este archivo** y comparan **todos** los tipos de paquete,
todos los largos y todos los enums compartidos con los de Python, en las dos
direcciones. Agregar algo en un solo lado rompe el test — que es exactamente
lo que hace falta, porque ya pasó: `CMD_FLAG_SIGNAL` existió varios commits
solo acá y la Raspberry Pi nunca supo mandarlo.

## Qué falta calibrar

Todo lo marcado con `TODO` en el código:

- Umbrales de `ClassifyColor()` para el TCS34725 **trasero**: hoy comparte los
  del delantero, un supuesto sin verificar. Los del delantero sí están
  recalibrados contra 970 muestras reales (ver
  [`../calibracion/color/`](../calibracion/color/)).
- `kDarkThreshold` de los QTR, con la luz del salón de competencia (ver
  [`../calibracion/reflectancia/`](../calibracion/reflectancia/)).

Los ángulos de la pinza (`kClawOpenDeg`, `kClawClosedLlaveDeg`,
`kClawClosedBanderaDeg`) **ya están calibrados** con
`pruebas-platformio/06-calibracion-gripper/`. El robot tiene un solo servo, así
que no hay ángulos de elevación que calibrar.
