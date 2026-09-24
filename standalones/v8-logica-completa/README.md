# v8-logica-completa

Banco de validación de los 12 pasos de [`docs/logica-athena.md`](../../docs/logica-athena.md). Se construye por hitos a partir de una copia de `v7-mision-completa-camara`; **v6 y v7 no se tocan**. Cuando esté depurado, lo que funcione se promueve a `firmware-esp32/` y `raspberry-pi/`.

| Hito | Qué | Estado |
|---|---|---|
| M0 | Copia de v7, compila | hecho (sin probar en robot) |
| M1 | Pasos 1–6 con protección de borde mínima (QTR derecho) | flasheado, en pruebas |
| M2 | Búsqueda y centrado (paso 7), cámara simulada a mano | pendiente |
| M3 | Ajuste con ToF y cierre con cámara + ToF (8–9) | pendiente |
| M4 | QTR: medir el izquierdo y activar el borde | pendiente |
| M5 | Retorno y suelta (10–12) | pendiente |
| M6 | Lado Pi: script nuevo, servicio y logs | pendiente |
| M7 | Corrida completa en pista | pendiente |

Uso: `pio run -t upload --upload-port COM11` y `pio device monitor --port COM11` (log por UART). Los parámetros ajustables están al inicio de `src/main.cpp` (`Mission::`).
