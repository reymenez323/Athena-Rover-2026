# Detector de color

Sketch de banco, independiente de `../firmware/` (el de captura): lee el
sensor B en bucle, clasifica NEGRO/GRIS con un umbral fijo, imprime por
consola cada 200 ms, y enciende el LED RGB en rojo (negro) o verde (gris).

No manda ni recibe comandos por serial como `../firmware/` — no lo maneja
`../calibrar_ir.py`. Es de uso directo: subir y abrir el monitor.

## Compilar, subir y ver

```bash
pio run -t upload -t monitor
```

Mismo puerto y mismo criterio que `../firmware/`: `Serial` en UART0 por
defecto (sin USB nativo), así que es el puerto de siempre, sin nada
especial que ajustar.

## De dónde sale el umbral (145)

De las capturas reales en `../data_logs/` — `IR_NEGRO_2026-08-28_07-14-49.csv`
y `IR_GRIS_2026-08-28_07-17-36.csv`, 740 muestras del sensor B:

| Superficie | Media | Desv. estándar |
|---|---:|---:|
| NEGRO | 165.2 | 13.1 |
| GRIS | 124.4 | 7.6 |

145 es el punto medio razonable entre las dos medias. Sigue siendo un
umbral fijo sacado de un log puntual, no una calibración en vivo — si
cambia la luz del lugar o se reposiciona el sensor, hay que revisarlo de
nuevo. Para el umbral que de verdad usa el robot, manda la calibración en
vivo de `pruebas-platformio/01-mantente-en-cuadro/` y `02-cuadro-color-rgb/`
(recalibran en cada arranque); esto es solo una herramienta de banco para
ver el clasificador funcionando rápido.

## Por qué solo decide el sensor B (por ahora)

En las dos capturas de arriba, el sensor A dio **0 en las 740 muestras**,
sin excepción. La causa no era ruido ni un cable suelto: estaba en GPIO35,
que en el ESP32-S3 **no tiene hardware de ADC**. `analogRead()` ahí siempre
falla con `Pin 35 is not ADC pin!` y devuelve 0. Ya se recableó a GPIO5 (sí
es ADC1 en el S3), pero como el umbral de arriba se calculó sin el sensor
A, este sketch lo sigue leyendo e imprimiendo solo como diagnóstico — no
participa en la clasificación todavía. Si quieres sumarlo, hay que volver a
capturar NEGRO/GRIS con `../calibrar_ir.py` (ahora sí con A dando datos
reales) y recalcular el umbral.

## Pines

Idénticos a `../firmware/src/main.cpp`, más el LED RGB:

| Señal | GPIO |
|---|:---:|
| `QTR_A_SENSOR` | 5 (recableado desde GPIO35 — no es pin de ADC en el S3) |
| `QTR_A_CTRL` / `QTR_B_CTRL` | 25 (compartido) |
| `QTR_B_SENSOR` | 4 |
| `GENERIC_IR` | 14 |
| LED RGB — R | 39 |
| LED RGB — G | 38 |
| LED RGB — B | 3 |

> ⚠️ Polaridad del LED sin confirmar (mismo TODO que en `firmware-esp32/` y
> `pruebas-platformio/02-cuadro-color-rgb/`): si los colores salen
> invertidos, cambia `kCommonAnode` a `true` en `src/main.cpp`.
