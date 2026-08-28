# Calibración IR

Herramienta de banco para caracterizar un sensor de reflectancia
(QTRX-HD-01A analógico + QTRX-HD-01RC en modo RC, más un módulo IR genérico)
sobre distintas superficies, **antes** de fijar umbrales en el firmware o en
las pruebas de PlatformIO. No es parte del firmware de vuelo ni de
`pruebas-platformio/`.

## ⚠️ Esta NO es la placa del rover

El sketch de `firmware/` corre en un **ESP32 WROVER-E (Freenove)** de banco,
no en el ESP32-S3 del chasis. Los pines (`GPIO35`, `GPIO25`, `GPIO4`,
`GPIO26`, `GPIO14`) solo existen en esa placa; no tienen relación con
[`../hardware/conexiones-esp32-s3.md`](../hardware/conexiones-esp32-s3.md).
Es intencional: permite mover el sensor a mano sobre distintas superficies
sin desarmar el robot.

## Dos partes

| Parte | Qué hace |
|---|---|
| `firmware/` | Sketch del ESP32 de banco. Solo responde: recibe el comando `'R'` por serial y contesta `DATA,<analog>,<rc>,<generic>`. Usa la librería `QTRSensors` de Pololu (única excepción del proyecto a "cero dependencias" — es una herramienta de banco, no el firmware de vuelo). |
| `calibrar_ir.py` | Script de Python que orquesta la prueba: pide moverse a cada punto, manda `'R'` repetidamente, y guarda todo en un `.csv` dentro de `data_logs/`. |

## Cómo usar

1. Conectar el ESP32 WROVER-E y subir el sketch:
   ```bash
   cd firmware
   pio run -t upload
   ```
2. Instalar la dependencia de Python (una sola vez):
   ```bash
   pip install -r requirements.txt
   ```
3. Correr la calibración, una vez por superficie:
   ```bash
   python3 calibrar_ir.py --puerto COM5 --superficie NEGRO --puntos 4 --muestras 60
   python3 calibrar_ir.py --puerto COM5 --superficie GRIS --puntos 6 --muestras 50 --intervalo-ms 300
   ```
   Antes de cada punto (incluido el primero), el script cuenta regresivamente
   `--pausa-s` segundos (10 por defecto): es la ventana para mover el sensor
   a la posición o superficie que toca medir.
4. El resultado queda en `data_logs/IR_<SUPERFICIE>_<fecha>_<hora>.csv`.

Ver `python3 calibrar_ir.py --help` para todas las opciones.

## Formato del CSV

Los metadatos de la corrida van como líneas de comentario (`#`) al inicio —
sigue siendo CSV válido, `pandas.read_csv(archivo, comment="#")` los ignora
solo:

```
# ROVER IR SENSOR CHARACTERIZATION
# Date: 2026-08-22 17:22:10.478004
# Surface: BLACK
# Number of points: 4
# Samples per point: 60
# Sample interval: 250 ms
# Movement time between points: 10 s
surface,point,sample,analog_QTRX,RC_QTRX_us,generic_IR
BLACK,1,1,4095,1055,1
...
```

| Columna | Qué es |
|---|---|
| `analog_QTRX` | Lectura del QTRX-HD-01A analógico, 0–4095 (ADC de 12 bits), promediada 8x por el sketch. |
| `RC_QTRX_us` | Tiempo de descarga del QTRX-HD-01RC en modo RC, en microsegundos (tope 5000 µs). |
| `generic_IR` | Lectura digital (0/1) de un módulo IR genérico aparte, solo para comparar. |

## Por qué esto no reemplaza la calibración en vivo de las pruebas

`pruebas-platformio/01-mantente-en-cuadro/` y `02-cuadro-color-rgb/`
calibran el QTR **en cada arranque**, sobre el ESP32-S3 y a la altura real
de montaje en el chasis — eso sigue siendo lo que decide el umbral que
efectivamente usa el robot. Esta herramienta es para entender **cómo se
comporta el sensor** (analógico vs. RC vs. IR genérico, negro vs. gris)
antes de decidir cuál de los tres métodos usar y con qué margen, no para
generar el umbral final.

Análisis ya hecho con los primeros dos logs de esta herramienta: el negro
satura consistentemente cerca del máximo del ADC analógico (~4060 de media,
desviación ~135), pero el gris varía mucho según la altura/ángulo con que se
sostuvo el sensor a mano (~3400–3960 de media según el punto), llegando a
solaparse con el negro en el punto más alto. Por eso la calibración en vivo
del ESP32-S3 (a altura fija de montaje) es la que manda para el umbral real.
