# 06 — Calibración interactiva del servo del gripper

Sketch para encontrar a mano los ángulos del servo de la pinza (canal 0 del
PCA9685, ver `hardware/conexiones-esp32-s3.md`) que hacen falta para agarrar
el **cilindro** (el asta de la bandera) y la **llave** (que es un cubo) del
reto. No decide nada por su cuenta: mueve el servo por comandos del monitor
serial y tú anotas los ángulos que veas correctos.

## Por qué existe

`04-servos-a-cero/` deja el gripper en 0° como referencia mecánica fija,
pero no sirve para explorar ángulos. Este sketch es la herramienta que se
usó para llenar de números reales las constantes de
`firmware-esp32/src/main.cpp` (sección `GripperTask`).

## Resultado ya integrado

Estos son los ángulos medidos con este sketch, ya copiados a
`firmware-esp32/src/main.cpp` (`kClawOpenDeg`, `kClawClosedLlaveDeg`,
`kClawClosedBanderaDeg`):

| Objeto | Abierto | Cerrado |
|---|---|---|
| Cilindro (asta de la bandera) | 0° | 65° |
| Llave (cubo) | 0° | 120° |

Como los dos "cerrado" son distintos, `GripperAction` ya no tiene un `CLOSE`
genérico: tiene `CLOSE_LLAVE` y `CLOSE_BANDERA` (ver `firmware-esp32/src/
main.cpp` sección `[2]` y `raspberry-pi/src/athena/protocol.py`). Si vuelves
a calibrar (otro gripper, otro objeto), actualiza esas constantes allá
también.

Sigue pendiente `kLiftUpDeg`/`kLiftDownDeg` del segundo servo (elevación,
canal 1) — este sketch, tal como está, solo controla un canal a la vez
(seleccionable con `canal <n>`), así que sirve igual para calibrarlo cuando
ese servo esté montado.

## Antes de encender

Igual que en `04-servos-a-cero/`: la primera vez, monta el gripper
**desacoplado** del mecanismo. Al arrancar, el sketch manda el servo a 0° —
si el cuerno ya está atornillado contra un tope mecánico, ese salto inicial
lo fuerza. Confirmado que 0° es un punto seguro, sí conviene calibrar con el
mecanismo ya acoplado: solo así se siente cuándo la pinza agarra de verdad.

## Cómo se usa

1. `pio run -t upload -t monitor` (o el botón equivalente de la extensión).
2. En el monitor serial (115200 baudios), escribe un comando y Enter:

| Comando | Efecto |
|---|---|
| `c <grados>` | Servo al ángulo absoluto indicado (0-180), en el canal seleccionado |
| `c+` / `c-` | Nudge de +paso / -paso grados |
| `canal <n>` | Cambia el canal del PCA9685 controlado (0-15, arranca en 0) |
| `scan` | Prueba los 16 canales en orden, moviendo cada uno para ver a simple vista cuál mueve tu servo |
| `paso <n>` | Cambia el tamaño del nudge (arranca en 5°) |
| `0` | El servo de vuelta a 0° (reset de seguridad) |
| `?` | Reimprime el menú de ayuda |
| *(línea vacía)* | Reimprime el estado actual |

## Si no sabes en qué canal está el servo

Envía `scan`. El sketch prueba los 16 canales del PCA9685 en orden, moviendo
cada uno de 0° a 180° y de vuelta mientras imprime `[Scan] Canal N...` por
consola. Es bloqueante a propósito (unos 25 segundos en total) — mira el
gripper físico y anota el número que imprimió justo cuando lo viste moverse.
Después, `canal <n>` con ese número deja `c`/`c+`/`c-`/`0` apuntando ahí.

Cada comando imprime el estado completo después de aplicarse — el número
que hay que anotar siempre es el último que aparece en pantalla, por
ejemplo:

```
[Estado] canal=0   pinza=65 grados   paso=5 grados   PCA9685=OK
```

> `monitor_filters = send_on_enter` en `platformio.ini` hace que el monitor
> de PlatformIO también sirva para escribir, no solo para leer logs. Si usas
> otra herramienta de monitor serial y no manda salto de línea al enviar, el
> firmware igual ejecuta el comando tras ~200 ms sin bytes nuevos (ver
> `LINE_IDLE_TIMEOUT_MS` en el código).

## Flujo seguido (repetir para cilindro y para llave)

1. Con `paso 2` o `paso 3` (nudges finos), cierra `c` de a poco hasta que la
   pinza encierre el objeto sin tocarlo todavía — anota ese ángulo como
   **"abierta"**.
2. Sigue cerrando hasta sentir que agarra firme, **sin forzar** el servo
   contra el objeto (un servo forzado calienta y con el tiempo pierde
   torque) — anota ese ángulo como **"cerrada"**.
