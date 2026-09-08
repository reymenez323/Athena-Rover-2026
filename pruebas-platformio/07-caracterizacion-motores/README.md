# 07 — Caracterización interactiva de los 4 motores

Sketch para accionar **cada motor por separado** (delantero izquierdo,
delantero derecho, trasero izquierdo, trasero derecho) — adelante y atrás —
y dejar un **log** con lo que se probó, para poder compartirlo y confirmar
(o corregir) el sentido de giro y el cableado de cada uno.

## Por qué existe

`03-motores-adelante/` ya prueba los 4 motores, pero todos a la vez, solo
hacia adelante, a fondo, y arranca solo (no hay forma de pararlo por
software). Sirve para validar el cableado en bloque, pero no para
caracterizar motor por motor ni para ver atrás.

Este sketch es la herramienta para eso: no mueve nada hasta que le mandas un
comando, controla un motor a la vez (o los 4 juntos), en cualquier
dirección, a cualquier duty, y deja una línea `[LOG]` por cada cambio de
estado — el mismo tipo de evidencia que hizo falta para detectar que el
motor trasero izquierdo giraba al revés de los otros tres (ver el aviso en
[`hardware/conexiones-esp32-s3.md`](../../hardware/conexiones-esp32-s3.md#motores--2-l298n)
y la nota en `kMotorFL` de `03-motores-adelante/src/main.cpp`, que este
sketch reutiliza tal cual).

## Antes de encender

Pon el robot sobre un soporte con las 4 ruedas al aire, o en el piso con
espacio libre alrededor. A diferencia de `03-motores-adelante/`, este
sketch **no arranca solo**: los 4 motores quedan detenidos hasta que mandas
un comando, y `alto` los detiene en cualquier momento. Aun así, con `todos`
seleccionado y duty alto el robot sí puede desplazarse — no lo dejes sobre
el piso sin espacio.

## Cómo se usa

1. `pio run -t upload -t monitor` (o el botón equivalente de la extensión).
2. En el monitor serial (115200 baudios), escribe un comando y Enter:

| Comando | Efecto |
|---|---|
| `motor <fl\|fr\|rl\|rr\|todos>` | Selecciona el motor objetivo de los comandos de abajo. Arranca en `todos`. |
| `adelante [duty]` | Motor(es) seleccionado(s) hacia adelante. Sin `duty`, usa el valor por defecto (`duty <n>`, arranca en 200/255). |
| `atras [duty]` | Igual, hacia atrás. |
| `alto` | Detiene el motor(es) seleccionado(s). |
| `duty <n>` | Cambia el duty por defecto (0-255) que usan `adelante`/`atras` sin argumento. |
| `secuencia [duty] [ms]` | Rutina automática: prueba los 4 motores **uno a la vez**, adelante `ms` milisegundos (por defecto 1500), pausa, atrás `ms`, pausa, y sigue con el siguiente motor — en orden FL, FR, RL, RR. Bloqueante a propósito. |
| `?` | Reimprime el menú de ayuda. |
| *(línea vacía)* | Reimprime el estado completo de los 4 motores. |

Cada cambio de dirección imprime una línea así, pensada para ser fácil de
leer o filtrar:

```
[LOG] t_ms=12345 motor=FL dir=ADELANTE duty=200/255
```

## Cómo generar y compartir el log

`platformio.ini` tiene `monitor_filters = send_on_enter, log2file`: la
sesión completa del monitor (comandos que escribes + todo lo que imprime el
firmware, líneas `[LOG]` incluidas) queda guardada automáticamente en un
`.log` con marca de tiempo, dentro de esta misma carpeta
(`pruebas-platformio/07-caracterizacion-motores/`). Ese archivo ya está
ignorado por git (`.gitignore` de esta carpeta y el `*.log` global) — no
hace falta limpiarlo antes de hacer commit.

Flujo sugerido para caracterizar los 4 motores de una sola vez:

1. Con las ruedas al aire, correr `secuencia` (o `secuencia 200 2000` para
   ajustar duty/duración).
2. Observar físicamente cada rueda mientras corre: ¿gira en el sentido
   correcto para "adelante" del robot? ¿Las 4 tardan lo mismo en arrancar,
   o alguna arrastra?
3. Cerrar el monitor (o mandar Ctrl+C) para que PlatformIO termine de
   escribir el `.log`.
4. Compartir ese archivo — junto con qué motor(es) giraron mal, si alguno —
   para revisar si el cableado/orientación necesita el mismo tipo de ajuste
   que ya tiene `kMotorFL` (intercambio de IN1/IN2), o si el log confirma
   que los 4 están correctos.

Si algún motor resulta invertido, el arreglo es el mismo que ya documenta
`03-motores-adelante/src/main.cpp` y `hardware/conexiones-esp32-s3.md`:
intercambiar el ORDEN de sus dos GPIO en la definición de `Motor`
correspondiente (`kMotorFL`/`kMotorFR`/`kMotorRL`/`kMotorRR`), no agregar un
flag de inversión — y replicar el cambio en los demás sketches que
controlan motores (ver la lista en `hardware/conexiones-esp32-s3.md`).

> `monitor_filters = send_on_enter` hace que el monitor de PlatformIO
> también sirva para escribir, no solo para leer logs. Si usas otra
> herramienta de monitor serial, el firmware igual ejecuta el comando tras
> ~200 ms sin bytes nuevos aunque no mande salto de línea al enviar (ver
> `LINE_IDLE_TIMEOUT_MS` en el código).
