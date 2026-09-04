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
| PCA9685 | 1 | 2 servos del gripper, por I2C |
| TCS34725 | 2 | Color del piso, delantero y trasero |
| QTRX-HD-01A | 2 | Reflectancia delantera, izquierda y derecha |
| LED RGB | 1 | Identificación de equipo (lo exige el reglamento); antes eran 2 LED discretos rojo/azul |

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
| Gripper | 3 | 1 | 50 ms | Los 2 servos vía PCA9685 |
| Reflectance | 3 | 0 | 20 ms | Los 2 QTRX |
| ColorSensors | 2 | 0 | 100 ms | Los 2 TCS34725 |
| TeamLed | 1 | 0 | 250 ms | LED de equipo |

Las tareas de control crítico van fijadas al **núcleo 1**; las de sensado y
estado al **núcleo 0**. Así un sensor lento no le quita tiempo de CPU al lazo
de control de los motores.

## Aislamiento entre tareas

No existe un "estado global del robot". Cada tarea es dueña de su hardware y de
sus datos, y los publica por colas explícitas de FreeRTOS.

```
RPi ──USB──> SerialTask ──> motorCmdQueue   (1, overwrite) ──> MotorTask
                        ──> gripperCmdQueue (4, FIFO)      ──> GripperTask
                        ──> ledCmdQueue     (1, overwrite) ──> LedTask

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

El formato completo está documentado en la sección `[3]` de `main.cpp`. El lado
Python está en `raspberry-pi/src/athena/protocol.py`, y hay un test que lee
**este archivo** y compara los números, de modo que los dos lados no se puedan
desincronizar en silencio.

## Qué falta calibrar

Todo lo marcado con `TODO` en el código:

- `kLiftUpDeg`/`kLiftDownDeg` del segundo servo (elevación) — **no aplica
  todavía**: ese servo no está montado en el robot, así que todo lo que lo
  toca (constantes, envío inicial, casos `RAISE`/`LOWER` de `GripperTask`)
  quedó comentado en `main.cpp` hasta que se agregue el hardware. Cuando
  esté, hacerlo con el servo **desacoplado** del mecanismo, para no forzarlo
  contra un tope. Los ángulos de la pinza (`kClawOpenDeg`,
  `kClawClosedLlaveDeg`, `kClawClosedBanderaDeg`) ya están calibrados con
  `pruebas-platformio/06-calibracion-gripper/`.
- Umbrales de `ClassifyColor()` para el TCS34725, sobre la pista real.
- `kDarkThreshold` de los QTR, con la luz del salón de competencia.
