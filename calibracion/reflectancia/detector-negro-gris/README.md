# Detector NEGRO/GRIS

Sketch de banco, independiente de `../firmware/` (el de captura): lee los dos
QTRX-HD-01A (izquierdo GPIO1, derecho GPIO2) en bucle, clasifica cada uno
por separado, imprime por consola cada 200 ms y lo muestra en la tira
WS2812B (GPIO 39):

| LED | Muestra |
|---|---|
| 0–3 (izquierda) | QTR izquierdo |
| 4–7 (derecha) | QTR derecho |

| Color | Significado |
|---|---|
| morado | NEGRO (borde) |
| apagado | GRIS (pista) |
| rojo tenue | sin señal: el sensor está pegado al tope del ADC |

No manda ni recibe comandos por serial como `../firmware/` — no lo maneja
`../calibrar_ir.py`. Es de uso directo: subir y abrir el monitor.

```bash
pio run -t upload -t monitor
```

`Serial` va en UART0 (sin USB nativo), el puerto de siempre.

## Cómo mide (cambió el 2026-10-01)

Antes usaba `QTRSensors` con el emisor siempre encendido y un umbral sobre el
valor crudo del sensor B (2910). Ya no: ahora usa la **misma secuencia que el
robot** (v8/v9): emisor apagado ≥ 1 ms → lectura `off`; emisor encendido →
lectura `on`; `dif = on − off`, y `|dif| < umbral` = NEGRO. Con el sensor
derecho se midió negro ≈ 1–8 y gris ≈ 76–87, umbral 40.

La consola imprime `off`, `on` y `dif` de cada sensor.

## ⚠️ El umbral del izquierdo no está calibrado

El 40 se midió solo con el derecho; el izquierdo se daba por averiado. Si ya
funciona, hay que medir su propio umbral: `kUmbralIzquierdo` en
`src/main.cpp` es hoy una copia del derecho. Procedimiento: sostén los
sensores a la altura real de montaje sobre el gris de la pista y luego sobre
la cinta negra, mira la columna `dif` de cada uno y fija el umbral entre los
dos rangos. Los CSV de `../data_logs/` (2026-08-28) son de la lectura cruda
vieja y no sirven para esto.

El módulo IR genérico no se usa. Sigue siendo una herramienta de banco: si
cambia la luz o se reposiciona el sensor, verifícalo de nuevo.
