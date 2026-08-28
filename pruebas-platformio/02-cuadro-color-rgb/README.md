# Prueba 02 — Cuadro + color + RGB

Combina la prueba [`01-mantente-en-cuadro`](../01-mantente-en-cuadro/) (código
sin tocar: los 2 QTR + los 2 L298N evitando la cinta negra) con los 2
TCS34725: mientras el robot se mantiene dentro del cuadro, cada sensor de
color clasifica lo que ve al pasar sobre las zonas de la pista. Como solo hay
**un** LED RGB en el chasis, este muestra el color que detecta el sensor
**delantero**; el trasero solo se reporta por el monitor serial.

Es un sketch de banco, independiente del firmware principal y de la prueba
01: sirve para validar el driver TCS34725 + el RGB con hardware real antes de
integrarlos a las 7 tareas de FreeRTOS de `firmware-esp32/`. Usa los mismos
pines documentados en
[`../../hardware/conexiones-esp32-s3.md`](../../hardware/conexiones-esp32-s3.md),
más el LED RGB (ver abajo, todavía no está en esa doc).

## Compilar y subir

```bash
pio run -t upload -t monitor
```

El monitor usa el puerto **UART** del DevKit (no el "USB" nativo), igual que
en la prueba 01.

## Motor invertido

Igual que en la prueba 01: con el cableado físico actual, la rueda trasera
izquierda (conectada a OUT1/OUT2 del L298N izquierdo, `kMotorFL` en el
código) gira al revés respecto a las otras tres. Se resuelve en la
definición de `kMotorFL` (`invert = true`), y `MotorApply()` lo aplica sin
importar la velocidad.

## Cableado nuevo — LED RGB

No estaba en `hardware/conexiones-esp32-s3.md` (esa doc solo documenta los 2
LED de equipo rojo/azul). Se usaron los únicos 3 GPIO que esa misma doc
marca libres para ampliaciones:

| Canal | GPIO ESP32-S3 |
|-------|:-------------:|
| R | **39** |
| G | **38** |
| B | **3** (strapping de JTAG — solo importa su nivel al encender/resetear; como salida normal después de bootear no da problema) |

**Polaridad sin confirmar todavía.** El código asume cátodo común (duty PWM
alto = canal más brillante). Si al probarlo los colores salen invertidos —por
ejemplo, "apagado" se ve más brillante que "rojo"— es ánodo común: cambiar
`kRgbCommonAnode` a `true` al principio de `src/main.cpp`, no hace falta
tocar nada más.

## Por qué NO se usó el dataset KNN del compañero

Se revisaron `PRUEBA_KNN_TCS34725_SIN_I2C.ino` y
`parametros_mejor_modelo.h` que trajo el equipo (un TCS34725 igual al
nuestro, dataset de 500 muestras, KNN con k=1, 98.4% de accuracy en su
validación cruzada). **No se integró tal cual** porque el TCS34725 normaliza
cada canal contra el canal C (luz total), y ese modelo se entrenó con
condiciones que probablemente no coinciden con las nuestras:

- Altura del sensor sobre la pista (el suyo, ~5 mm — la nuestra depende de
  cómo quedó montado en el chasis).
- Intensidad/ángulo del LED de iluminación del propio sensor.
- Luz ambiente del lugar donde se capturó el dataset.

Reusar su media/desviación estándar y sus 500 puntos de entrenamiento tal
cual, con un sensor en condiciones distintas, corre el riesgo real de
clasificar mal — no es un problema de que el sensor sea "diferente" (es el
mismo modelo), es que el dataset está atado a la geometría y luz con la que
se capturó.

Lo que **sí** es reutilizable de ese código si más adelante hace falta más
precisión que la de umbrales fijos:

- Las features (Rn, Gn, Bn normalizados + C) y la estandarización — mismo
  enfoque que ya usa `ClassifyColor()`, solo que con umbrales fijos en vez de
  un clasificador entrenado.
- El algoritmo KNN en sí (`clasificarKNN()`, `distancia2()`,
  `insertarVecino()`): son ~150 líneas sin dependencias, portables tal cual.

Para usarlo de verdad haría falta recapturar el dataset **con nuestros dos
TCS34725, montados en el chasis final, a la altura y con la iluminación con
la que va a correr en competencia** — no con los sensores sueltos en la mesa.

## Clasificación de color usada en esta prueba

`ClassifyColor()` es una copia exacta de la de `firmware-esp32/src/main.cpp`:
umbrales fijos sobre R/G/B normalizados contra C. **No están calibrados**,
son el mismo punto de partida documentado allá. Para ajustarlos:

1. Subir esta prueba y abrir el monitor serial.
2. Pasar cada sensor sobre las 4 superficies de la pista (negro, gris, rojo,
   azul, amarillo) y anotar los valores crudos `R/G/B/C` que imprime cada
   200 ms.
3. Ajustar los umbrales en `ClassifyColor()` (en este archivo, no en el
   firmware — copiar el cambio para allá una vez validado).

## Parámetros para ajustar

| Constante | Qué controla |
|---|---|
| `kForwardSpeed` / `kBackSpeed` / `kTurnSpeed` | Velocidad (0–100 %) en cada estado del cuadro |
| `kBackUpMs` / `kTurnMs` / `kTurnCornerMs` | Tiempos de la maniobra de borde |
| `MIN_USABLE_SPAN` / `FALLBACK_THRESHOLD` | Calibración de los QTR (ver prueba 01) |
| `COLOR_READ_MS` | Cada cuánto se lee cada TCS34725 (100 ms por defecto; no bajar de ~25 ms, el sensor integra 24 ms) |
| Umbrales dentro de `ClassifyColor()` | Qué cuenta como cada color |
| `kRgbCommonAnode` | Polaridad del LED RGB (ver arriba) |

## Qué mirar si no funciona

- **"sensor DELANTERO/TRASERO no responde"** por consola: revisar SDA/SCL de
  ese bus y las resistencias pull-up (4.7 kΩ a 3.3 V) — ver la nota de I2C en
  `hardware/conexiones-esp32-s3.md`.
- **El RGB no enciende o se queda en un color raro**: confirmar los 3 GPIO
  (39/38/3) y probar `kRgbCommonAnode` en el otro estado.
- **Bordes del cuadro**: mismos síntomas y causas que en la prueba 01.
