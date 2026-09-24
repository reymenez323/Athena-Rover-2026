# v8-logica-completa

Banco de validación de los 12 pasos de [`docs/logica-athena.md`](../../docs/logica-athena.md). Se construye por hitos a partir de una copia de `v7-mision-completa-camara`; **v6 y v7 no se tocan**. Cuando esté depurado, lo que funcione se promueve a `firmware-esp32/` y `raspberry-pi/`.

| Hito | Qué | Estado |
|---|---|---|
| M0 | Copia de v7, compila | hecho (sin probar en robot) |
| M1 | Pasos 1–6 con protección de borde mínima (QTR derecho) | probado: pasos 1–6 OK; borde da falsos positivos cerca del amarillo, desactivado hasta M4 |
| M2 | Búsqueda y centrado (paso 7), cámara simulada a mano | probado con cámara simulada: OK |
| M3 | Ajuste con ToF y cierre con cámara + ToF (8–9) | compila; en prueba |
| M4 | QTR: borde en todas las fases que se mueven | código listo, desactivado: falta recalibrar el umbral con amarillo y caja |
| M5 | Retorno y suelta (10–12) | compila, sin probar en robot |
| M6 | Lado Pi: script nuevo, servicio y logs | escrito (`avisar_bandera_v8.py`), sin probar con la cámara real |
| M7 | Corrida completa en pista | pendiente |

Uso: `pio run -t upload --upload-port COM11` y `pio device monitor --port COM11` (log por UART). Los parámetros ajustables están al inicio de `src/main.cpp` (`Mission::`).
