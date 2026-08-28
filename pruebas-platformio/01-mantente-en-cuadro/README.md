# Prueba 01 — Mantente en el cuadro

Robot que avanza dentro del cuadrado delimitado por cinta negra (fondo gris
de la pista) y, al detectar el borde con los sensores de reflectancia
delanteros, retrocede y gira hacia el lado contrario antes de seguir. Nunca
debe cruzar la cinta.

Es un sketch de banco, independiente del firmware principal
(`firmware-esp32/`): sirve para validar el comportamiento de borde con
hardware real antes de integrarlo a las 7 tareas de FreeRTOS. Usa los mismos
pines documentados en [`../../hardware/conexiones-esp32-s3.md`](../../hardware/conexiones-esp32-s3.md),
así que corre tal cual sobre el chasis ya cableado (solo necesita los 2 QTR
y los 2 L298N — no requiere PCA9685 ni TCS34725 para esta prueba).

## Compilar y subir

```bash
pio run -t upload -t monitor
```

El monitor usa el puerto **UART** del DevKit (no el "USB" nativo), igual que
el firmware principal — ver `platformio.ini`.

## Cómo se calibra

Al encender, los dos LED de equipo parpadean juntos durante 3 segundos: es
la señal para pasar el robot **a mano** sobre la cinta negra y el piso gris
varias veces, cubriendo los dos sensores. El sketch se queda con el mínimo y
el máximo que vio cada sensor y usa el punto medio como umbral.

Por qué calibra en cada arranque en vez de usar un número fijo: se analizaron
los dos logs de calibración que trajo el equipo
(`IR_BLACK_2026-08-22_17-22-08.txt`, `IR_GREY_2026-08-22_17-05-14.txt`,
columna `analog_QTRX`, ADC de 12 bits):

| Superficie | Media | Desv. estándar | Rango por punto |
|---|---:|---:|---|
| Negro (cinta) | 4060 | 135 | 4033 – 4091 |
| Gris (piso) | 3740 | 337 | 3397 – 3960 |

El negro satura consistentemente cerca del máximo del ADC (4095), como se
espera del QTR. El gris es bastante más bajo en promedio, **pero uno de los
6 puntos medidos (media 3960) se mete dentro del rango del negro** — casi
seguro porque el sensor se sostuvo a mano a distinta altura/ángulo entre
puntos, y la reflectancia que ve un QTR depende muchísimo de la distancia a
la superficie, no solo del color. Un umbral fijo copiado de ese log sería
frágil. Montado en el chasis, a altura fija sobre la pista, el contraste
real debería ser más limpio — pero para no apostarlo todo a eso, el sketch
recalibra cada vez que arranca.

Si en esos 3 segundos un sensor no ve suficiente contraste (por ejemplo, se
olvidó pasarlo sobre la cinta), se avisa por consola y ese sensor usa el
umbral de reserva `3700` — el punto medio razonable entre el grueso del gris
medido (<3800 en 5 de los 6 puntos) y el piso del negro (>4030).

## Parámetros para ajustar

Todos están al principio de `src/main.cpp`, con comentarios:

| Constante | Qué controla |
|---|---|
| `kForwardSpeed` / `kBackSpeed` / `kTurnSpeed` | Velocidad (0–100 %) en cada estado |
| `kBackUpMs` | Cuánto retrocede antes de girar |
| `kTurnMs` | Cuánto gira cuando dispara un solo sensor |
| `kTurnCornerMs` | Cuánto gira cuando disparan los dos a la vez (esquina) |
| `MIN_USABLE_SPAN` | Contraste mínimo para aceptar la calibración en vivo |
| `FALLBACK_THRESHOLD` | Umbral de reserva si la calibración no sirvió |

## Qué mirar si no funciona

- Si el robot gira hacia el mismo lado sin parar: revisar el sentido de giro
  de las ruedas de ese lado en `MotorApply` (motor invertido físicamente).
- Si nunca detecta el borde: abrir el monitor y ver los valores crudos
  impresos cada 200 ms; comparar con el umbral calibrado que se imprime al
  final de `RunCalibration()`.
- Si detecta borde en todas partes (incluso sobre gris): la calibración no
  vio suficiente contraste — repetir el arranque pasando el sensor de forma
  más deliberada sobre ambas superficies durante el parpadeo de los LED.
