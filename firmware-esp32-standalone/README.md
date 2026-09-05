# Firmware ESP32-S3 AUTÓNOMO — Athena Rover 2026 (sin Raspberry Pi)

Variante de [`../firmware-esp32/`](../firmware-esp32/) pensada para correr la
**demostración sin ninguna computadora externa**: todo — percibir, decidir y
mover — corre dentro del ESP32-S3. No hay enlace serial con la Raspberry Pi,
no hay cámara, no hay `run_rover.py`. Se enciende el robot y hace su ronda
solo.

**Todo el firmware vive en un solo archivo:** `src/main.cpp`, igual que la
versión con Raspberry Pi. Mismo hardware, mismos drivers I2C escritos a mano
(PCA9685, TCS34725); lo que cambia es que la máquina de estados de
`raspberry-pi/src/athena/decision.py` se portó a C++ y corre como una tarea
más de FreeRTOS (`MissionTask`), en vez de vivir en Python al otro lado de un
cable USB.

## Reinicios inesperados en banco: sospechar de la batería primero

Si el monitor serial muestra ráfagas de `[E][Wire.cpp:513] requestFrom():
i2cRead returned Error ...` seguidas de que el reloj (los `[ms]` a la
izquierda de cada línea) **vuelve a empezar cerca de 0**, el ESP32-S3 se
está reiniciando solo — no es (solo) un fallo de lectura I2C.

Con el USB nativo como único puerto (`Serial` = `DEBUG_LINK`), cada reset
reenumera el dispositivo USB y el monitor tarda un instante en reconectar;
los primeros logs de ese arranque —incluido el motivo del reinicio— se
pierden en esa ventana. `setup()` ahora imprime explícitamente
`[Setup] Motivo del ultimo reinicio: ...` (vía `esp_reset_reason()`) y
espera hasta 1.5 s a que el monitor reconecte antes de imprimir, así que
una captura nueva del log debería mostrarlo.

Si dice **BROWNOUT**, es la batería: el voltaje cae debajo del mínimo del
ESP32-S3 al energizar los sensores/PCA9685 y el chip se reinicia por su
cuenta. Es el sospechoso más probable si ya sabes que la batería está baja
— y explica de paso por qué los sensores "no responden" de forma
intermitente (el bus queda en un estado raro justo cuando el regulador se
desploma a medio arranque). Prueba alimentando el ESP32-S3 solo desde el
cable USB (sin la batería) para confirmar: si los reinicios y los errores
de I2C desaparecen, el firmware está bien y lo que hace falta es cargar o
cambiar la batería.

## Bus I2C nº0 compartido: por qué hay un mutex

`GripperTask` (PCA9685), `ColorSensorTask` (TCS34725 delantero) y
`TofSensorTask` (VL53L1X) viven los tres en el mismo bus I2C nº0 (`Wire`),
pero corren en tareas de FreeRTOS distintas — Gripper en el núcleo 1,
Color y ToF en el núcleo 0. `TwoWire` no es segura para usarse desde varias
tareas a la vez, y con dos núcleos de por medio dos de esas tareas pueden
estar ejecutando una transacción I2C **al mismo tiempo**, no solo
intercaladas por el planificador. Eso corrompe el bus de forma
intermitente: exactamente los "problemas de I2C" que se ven al mezclar el
ToF con el sensor de color.

La solución es `g_i2c0Mutex`: cada tarea toma este mutex antes de tocar el
bus 0 y lo suelta apenas termina, así nunca hay dos transacciones activas
al mismo tiempo. El bus 1 (`Wire1`, el TCS34725 trasero) no lo necesita
porque nadie más lo usa. Con timeout corto (50 ms, no `portMAX_DELAY`): si
no se consigue el bus a tiempo, esa operación se da por fallida esta vuelta
y se reintenta en la siguiente, mismo criterio que ya usan los drivers de
este archivo.

## Interruptor de movimiento

`Mission::kMotionEnabled` en `src/main.cpp` decide si `MissionTask` mueve algo:

- **`true` (como está ahora):** misión completa — motores y gripper.
- **`false`:** el robot **no mueve motores ni gripper**; solo sigue leyendo
  los sensores de color y reflectancia y mostrando la zona de piso por el LED
  (ver más abajo). Útil para probar la identificación de línea en el banco, o
  para desactivar el robot sin desconectarle nada.

> Estuvo en `false` un tiempo, mientras se investigaba por qué el robot se
> reiniciaba solo. La causa resultó ser un **problema de conexión**, no la
> batería baja que se sospechó al principio. Resuelto eso, el movimiento
> quedó habilitado de nuevo.

## Qué se puede hacer sin Raspberry Pi, y qué no

El ESP32-S3 por sí solo **no tiene cámara**: puede leer sensores de contacto
y de color de piso, pero no puede reconocer la forma ni el color de un
objeto lejano. Con eso alcanza para:

- ✔ **Identificar las zonas de piso** negro, amarillo, rojo y azul (mismos
  sensores TCS34725 y mismos umbrales calibrados que `firmware-esp32/`).
- ✔ **Cargar y depositar la llave especificada**: se asume que un operador
  la coloca a mano en la pinza abierta antes de encender el robot (la misma
  suposición que ya hace `decision.py` en su fase `INICIO`) — el firmware la
  sujeta, la lleva a la zona amarilla y la suelta solo.
- ✔ **Trasladar la bandera** una vez agarrada: cerrarla, volver hacia la
  zona propia (reconocida por el color de piso) y soltarla ahí.
- ✘ **Verificar que lo que agarra es específicamente la bandera del equipo
  contrario.** Sin cámara no hay forma de distinguir "bandera roja" de
  "bandera azul" ni de "cualquier otro objeto" a distancia. Este firmware
  usa el VL53L1X (el telémetro delante del gripper) como sustituto: durante
  la fase de búsqueda barre el área y cierra la pinza sobre lo primero
  sólido que encuentra a corta distancia. **Es una heurística, documentada
  a propósito — no una detección real.** Para eso hace falta la cámara +
  Edge Impulse de `raspberry-pi/src/athena/ei_flag_detector.py`, que es lo
  que usa `firmware-esp32/` junto con `run_rover.py`.

## LED RGB: indicador puro de línea/zona

El LED **solo** dice sobre qué está parado el robot ahora mismo — ya no
indica equipo ni "veo la bandera":

| Piso bajo el sensor delantero | LED |
|--------------------------------|-----|
| Gris (zona neutra) / sin lectura válida | Apagado |
| Amarillo | Amarillo fijo |
| Rojo | Rojo fijo |
| Azul | Azul fijo |
| Negro (borde) | Destello alternando rojo/azul (~1.7 Hz) |

## Selector de equipo (TEAM_SELECT)

Sin Raspberry Pi que le diga `--equipo rojo` o `--equipo azul` por línea de
comandos, el equipo se elige con un **puente físico a GND** en
`Pins::TEAM_SELECT` (GPIO40, libre en este robot — ver el historial de
`firmware-esp32/` sobre por qué):

| GPIO40             | Equipo |
|--------------------|--------|
| Puenteado a GND    | ROJO   |
| Sin conectar (pull-up interno) | AZUL |

Se lee **una sola vez**, en `setup()`. Cambiarlo exige reiniciar el ESP32,
que de todas formas ya se hace entre rondas.

## Secuencia de arranque

1. Al energizar, el robot espera quieto `Mission::kStartupDelayMs` (3 s por
   defecto) — tiempo para que el operador **coloque la llave a mano en la
   pinza abierta** y ubique el robot en la pista. El LED ya muestra la zona
   de piso bajo el sensor delantero desde este momento.
2. `ASEGURAR_LLAVE`: cierra la pinza sobre la llave.
3. `BUSCAR_ZONA_NEUTRA`: avanza recto hasta pisar la zona amarilla.
4. `DEPOSITAR_LLAVE`: abre la pinza.
5. `BUSCAR_BANDERA`: barrido de giro + avance en arco, monitoreando el
   VL53L1X (ver limitación arriba).
6. `AGARRAR_BANDERA`: cierra la pinza sobre lo que haya encontrado.
7. `RETORNAR_GIRANDO` → `RETORNAR_AVANZANDO`: gira ~180° (tiempo fijo, **a
   calibrar en banco**) y avanza hasta pisar la zona del propio equipo.
8. `ENTREGAR`: abre la pinza. Misión terminada, motores detenidos.

En todo momento, **evadir el borde negro con los QTR pisa cualquier otra
fase** — idéntico a la prioridad 1 de `decision.py`: salirse de la pista
pierde la ronda de inmediato, así que esa regla no puede depender de que
nada más ande bien.

## Hardware

Igual que [`../firmware-esp32/README.md`](../firmware-esp32/README.md), más
el puente TEAM_SELECT. Las conexiones completas están en
[`../hardware/conexiones-esp32-s3.md`](../hardware/conexiones-esp32-s3.md).

| Componente | Cantidad | Para qué |
|------------|:--------:|----------|
| L298N | 2 | 4 motores de tracción |
| PCA9685 | 1 | 2 servos del gripper, por I2C |
| TCS34725 | 2 | Color del piso, delantero y trasero |
| VL53L1X | 1 | Telémetro delante del gripper — aquí también hace de "ojos" para la bandera |
| QTRX-HD-01A | 2 | Reflectancia delantera, izquierda y derecha |
| LED RGB | 1 | Indicador puro de línea/zona de piso |
| Puente TEAM_SELECT | 1 | GND = ROJO, abierto = AZUL |

## Compilar y subir

```bash
pio run              # compilar
pio run -t upload    # subir al ESP32-S3
pio device monitor   # ver los logs de misión (fases, sensores, watchdog)
```

## Qué falta calibrar antes de una demo real

- `kDarkThreshold` de los QTR — con la luz del salón.
- Los umbrales de `ClassifyColor()` — igual que en `firmware-esp32/`.
- `Mission::kDistanciaAgarreMm` — según qué tan cerca hace falta estar para
  que el VL53L1X vea confiablemente el poste de la bandera y no otra cosa.
- `Mission::kReturnTurnMs` — el giro de ~180° es a tiempo fijo (no hay
  odometría ni encoders): depende del peso real del robot y de la fricción
  de la pista. Es el mismo hueco que ya anota `decision.py::_retornar`.
- `Mission::kSpinBurstMs` / `kForwardBurstMs` — qué tan agresivo es el
  barrido de búsqueda contra qué tan bien alcanza a leer el ToF en cada
  posición.

## Diferencias clave con `firmware-esp32/`

| | `firmware-esp32/` | `firmware-esp32-standalone/` |
|---|---|---|
| Cerebro de la misión | Raspberry Pi (`decision.py` + `run_rover.py`) | ESP32-S3 (`MissionTask`) |
| Detección de la bandera contraria | Cámara + Edge Impulse (color real) | VL53L1X (heurística de proximidad) |
| Selección de equipo | `--equipo rojo/azul` por línea de comandos | Puente físico TEAM_SELECT |
| Enlace serial | Protocolo binario `[0xAA][TYPE][LEN]...` con la Pi | Ninguno — `Serial` solo para logs de depuración |
| Uso previsto | Clasificatoria completa | Demostración |
