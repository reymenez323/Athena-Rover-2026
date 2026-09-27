# 10 — Tira WS2812B (8 LED), banco de prueba

Prueba aislada de la tira de LED que reemplaza al LED RGB de equipo (ver
[`hardware/conexiones-esp32-s3.md`](../../hardware/conexiones-esp32-s3.md),
sección "LED de indicación"). Confirma que la soldadura, la alimentación por
el buck (5 V) y la señal en el GPIO 39 funcionan antes de integrarla a la
lógica de la misión en
[`standalones/v9-tira-sensor-trasero/`](../../standalones/v9-tira-sensor-trasero/).

## Cableado

| Pin de la tira | Va a |
|---|---|
| GND | GND común |
| VCC | Buck, 5 V |
| IN | GPIO 39 |

## Cómo usarlo

Consola por el puerto de programación (COM11 en la PC de Montse), 115200
baudios. Comandos, una letra + Enter:

| Tecla | Efecto |
|---|---|
| `r` / `b` | Tira entera en rojo / azul (colores de equipo) |
| `w` | `colorWipe` |
| `t` | `theaterChase` |
| `c` | `rainbowCycle` |
| `h` | `theaterChaseRainbow` |
| `1`-`8` | Enciende solo ese LED (verde) — confirma el orden físico |
| `x` | Apaga todo |

Los efectos largos (`c`, `h`) revisan el puerto en cada paso: cualquier tecla
nueva los corta al instante, sin esperar a que terminen solos.

Brillo fijo al 15 % (`kBrillo` en el código) mientras no se mida cuánto
aguanta la fuente con todo lo demás conectado — no subirlo sin medir.

Basado en el `strandtest` oficial de Adafruit:
https://github.com/adafruit/Adafruit_NeoPixel/blob/master/examples/strandtest/strandtest.ino
