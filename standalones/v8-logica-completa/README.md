# v8-logica-completa

Banco de validación de los 12 pasos de [`docs/logica-athena.md`](../../docs/logica-athena.md). Se construye por hitos a partir de una copia de `v7-mision-completa-camara`; **v6 y v7 no se tocan**. Cuando esté depurado, lo que funcione se promueve a `firmware-esp32/` y `raspberry-pi/`.

| Hito | Qué | Estado |
|---|---|---|
| M0 | Copia de v7, compila | hecho |
| M1 | Pasos 1–6 con protección de borde mínima (QTR derecho) | probado: pasos 1–6 OK; borde da falsos positivos cerca del amarillo, desactivado hasta M4 |
| M2 | Búsqueda y centrado (paso 7), cámara simulada a mano | probado con cámara simulada: OK |
| M3 | Ajuste con ToF y cierre con cámara + ToF (8–9) | probado con cámara simulada y con la real |
| M4 | QTR: borde en todas las fases que se mueven | código listo, desactivado: falta recalibrar el umbral con amarillo y caja |
| M5 | Retorno y suelta (10–12) | probado en corrida real: misión completa a los 71 s (2026-09-24) |
| M6 | Lado Pi: script nuevo, servicio y logs | probado con la cámara real (`avisar_bandera_v8.py` + visor) |
| M7 | Corrida completa en pista | en curso: 2 corridas completas el 2026-09-24, ajustando parámetros |

Uso: `pio run -t upload --upload-port COM11` y `pio device monitor --port COM11` (log por UART). Los parámetros ajustables están al inicio de `src/main.cpp` (`Mission::`).
