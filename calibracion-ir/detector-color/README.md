# Detector de color

Sketch de banco, independiente de `../firmware/` (el de captura): lee los
sensores A y B en bucle, clasifica la superficie entre 4 etiquetas con
umbrales fijos, imprime por consola cada 200 ms, y enciende el LED RGB.

No manda ni recibe comandos por serial como `../firmware/` — no lo maneja
`../calibrar_ir.py`. Es de uso directo: subir y abrir el monitor.

## Compilar, subir y ver

```bash
pio run -t upload -t monitor
```

Mismo puerto y mismo criterio que `../firmware/`: `Serial` en UART0 por
defecto (sin USB nativo), así que es el puerto de siempre, sin nada
especial que ajustar.

## Límite físico importante: el QTR no ve colores, ve reflectancia IR

Con los 5 colores ya capturados (`../data_logs/`, superficies NEGRO, GRIS,
ROJO, AZUL, AMARILLO), la media ± desviación estándar en cada sensor fue:

| Superficie | Sensor A | Sensor B |
|---|---:|---:|
| NEGRO | 3969 ± 257 | 2925 ± 23 |
| GRIS | 3504 ± 301 | 2867 ± 55 |
| AZUL | 2691 ± 361 | 2613 ± 181 |
| ROJO | 2587 ± 355 | 2677 ± 187 |
| AMARILLO | 2551 ± 650 | 2288 ± 291 |

**ROJO y AZUL quedan a menos de 110 unidades de distancia en el sensor A —
dentro del ruido de cada uno.** No es un umbral mal elegido: un QTR
analógico mide reflectancia infrarroja, no color visible, y estos dos no se
pueden separar de forma confiable con este sensor. Por eso el diseño de
vuelo usa el TCS34725 (sensor de color RGB de verdad) para las zonas de la
pista, y el QTR solo para el borde de la cinta negra. Este sketch reporta
`ROJO/AZUL (ambiguo)` en vez de fingir que sabe cuál de los dos es.

## Cómo clasifica (3 umbrales en cadena)

1. **Sensor A > 3100** → tier "oscuro" (es la separación más limpia de las
   cinco). Si no, tier "color".
2. Dentro de "oscuro": **A > 3700** → `NEGRO`, si no → `GRIS`.
3. Dentro de "color": **sensor B < 2450** → `AMARILLO` (su B es
   notablemente más bajo que rojo/azul); si no → `ROJO/AZUL (ambiguo)`.

Los 3 números salen de los promedios de la tabla de arriba, redondeados a
puntos medios razonables entre grupos — no son una calibración estadística
rigurosa, son umbrales fijos sacados de un log puntual. Si cambia la luz
del lugar o se reposiciona el sensor, verifícalos de nuevo contra la
superficie real. La calibración en vivo de
`pruebas-platformio/01-mantente-en-cuadro/` y `02-cuadro-color-rgb/`
(recalibran en cada arranque) sigue siendo la que manda para el robot de
verdad — esto es solo una herramienta de banco.

## LED RGB

| Clasificación | Color del LED |
|---|---|
| `NEGRO` | Rojo — igual que antes |
| `GRIS` | Verde — igual que antes |
| `AMARILLO` | Amarillo |
| `ROJO/AZUL (ambiguo)` | Púrpura — a propósito ni rojo ni azul puro, para no fingir una respuesta que el sensor no puede dar |

## Pines

Idénticos a `../firmware/src/main.cpp`, más el LED RGB:

| Señal | GPIO |
|---|:---:|
| `QTR_A_SENSOR` | 1 (= `QTR_LEFT_OUT` en el diseño de vuelo) |
| `QTR_A_CTRL` / `QTR_B_CTRL` | 42 (compartido, = `QTR_EMITTER_CTRL`) |
| `QTR_B_SENSOR` | 2 (= `QTR_RIGHT_OUT`) |
| LED RGB — R | 39 |
| LED RGB — G | 38 |
| LED RGB — B | 3 |

El módulo IR genérico no está en uso (no está cableado), así que este
sketch ya no lo lee ni lo imprime.

> ⚠️ Polaridad del LED sin confirmar (mismo TODO que en `firmware-esp32/` y
> `pruebas-platformio/02-cuadro-color-rgb/`): si los colores salen
> invertidos, cambia `kCommonAnode` a `true` en `src/main.cpp`.
