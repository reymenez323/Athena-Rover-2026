# Detector NEGRO/GRIS

Sketch de banco, independiente de `../firmware/` (el de captura): lee los
sensores A y B en bucle, clasifica NEGRO/GRIS, imprime por consola cada
200 ms, y enciende el LED RGB en rojo (negro) o verde (gris).

No manda ni recibe comandos por serial como `../firmware/` — no lo maneja
`../calibrar_ir.py`. Es de uso directo: subir y abrir el monitor.

## Compilar, subir y ver

```bash
pio run -t upload -t monitor
```

Mismo puerto y mismo criterio que `../firmware/`: `Serial` en UART0 por
defecto (sin USB nativo), así que es el puerto de siempre, sin nada
especial que ajustar.

## Por qué decide solo el sensor B

Una versión anterior de este sketch usaba el sensor A para separar
negro/gris (umbral 3700) y clasificaba mal la mayoría del gris como negro.
En vez de mover ese umbral a ojo, se probaron sistemáticamente varias
opciones contra las 740 muestras reales en `../data_logs/` —
`IR_NEGRO_2026-08-28_07-42-21.csv` (240 muestras) e
`IR_GRIS_2026-08-28_07-44-29.csv` (500 muestras) — buscando en cada caso el
umbral que menos errores comete:

| Método | Errores totales | Detalle |
|---|---:|---|
| Solo A (umbral 3700) | 80 / 740 | 66 negro→gris, 14 gris→negro |
| A + 0.8·B (combinado) | 69 / 740 | 58 negro→gris, 11 gris→negro |
| **Solo B (umbral 2922)** | **40 / 740** | **40 negro→gris, 0 gris→negro** |

B solo, con el umbral correcto, resultó mejor que A solo y que cualquier
combinación lineal de ambos que se probó (se recorrió un rango de pesos de
-3 a 3). Por eso el sketch descarta A para la decisión — se sigue leyendo e
imprimiendo, pero es puramente informativo.

## De dónde sale el umbral (2922)

Con el sensor B: NEGRO dio un rango de 2804–2940, GRIS dio 2457–2922. Se
solapan solo en la franja 2804–2922 — ahí es donde caen los 40 errores (y
son casi todos negro leído como gris, nunca al revés con este umbral). No
es ruido de captura concentrado en un punto raro: se revisó punto por
punto y el solape está repartido en los 4 puntos de la captura de NEGRO
por igual, así que es ruido real de esa franja del sensor, no un accidente
de medición.

## Si hace falta más margen

El siguiente paso razonable **no es mover el umbral otra vez** — ya es el
óptimo para estos datos. Es fijar la atenuación del ADC explícitamente. Ni
`../firmware/` ni este sketch llaman `analogSetPinAttenuation()` como sí
hace `firmware-esp32/src/main.cpp` (`ADC_11db` en los pines QTR); ahora
mismo ambos dependen de lo que el core Arduino-ESP32 use por defecto. Fijar
la atenuación a propósito podría ampliar el rango dinámico entre negro y
gris — pero eso requiere volver a capturar datos para confirmarlo, no es
algo que se pueda saber sin probar.

Sigue siendo un umbral fijo sacado de un log puntual, no una calibración
en vivo — si cambia la luz del lugar o se reposiciona el sensor, verifícalo
de nuevo contra la superficie real. La calibración en vivo de
`pruebas-platformio/01-mantente-en-cuadro/` y `02-cuadro-color-rgb/`
(recalibran en cada arranque) sigue siendo la que manda para el robot de
verdad — esto es solo una herramienta de banco.

## Pines

| Señal | GPIO |
|---|:---:|
| `QTR_A_SENSOR` | 1 (= `QTR_LEFT_OUT` en el diseño de vuelo — se lee, no decide) |
| `QTR_A_CTRL` / `QTR_B_CTRL` | 42 (compartido, = `QTR_EMITTER_CTRL`) |
| `QTR_B_SENSOR` | 2 (= `QTR_RIGHT_OUT` — el que decide) |
| LED RGB — R | 39 |
| LED RGB — G | 38 |
| LED RGB — B | 3 |

El módulo IR genérico no está en uso (no está cableado), así que este
sketch no lo lee.

> ⚠️ Polaridad del LED sin confirmar (mismo TODO que en `firmware-esp32/` y
> `pruebas-platformio/02-cuadro-color-rgb/`): si los colores salen
> invertidos, cambia `kCommonAnode` a `true` en `src/main.cpp`.
