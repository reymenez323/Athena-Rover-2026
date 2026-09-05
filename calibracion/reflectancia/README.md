# Calibración IR

Herramienta de banco para caracterizar 2 sensores de reflectancia (ambos
QTRX-HD-01A analógicos, más un módulo IR genérico) sobre distintas
superficies, **antes** de fijar umbrales en el firmware o en las pruebas de
PlatformIO. No es parte del firmware de vuelo ni de `pruebas-platformio/`.

## Mismo ESP32-S3, pero solo con los sensores conectados

El equipo usa un único microcontrolador en todo el proyecto — los sketches
de esta carpeta corren sobre el mismo ESP32-S3, no una placa aparte. Lo que
sí es distinto es el contexto: se suben solos, con los sensores bajo prueba
(y, para `detector-negro-gris/`, un LED RGB) conectados y **nada más** (sin
motores, sin PCA9685, sin TCS34725), así que los sensores se pueden mover a
mano sobre distintas superficies sin desarmar el robot.

| Señal | GPIO | Nota |
|---|:---:|---|
| `QTR_A_SENSOR` | **1** | QTRX-HD-01A, modo analógico — mismo GPIO que `QTR_LEFT_OUT` en [`../../hardware/conexiones-esp32-s3.md`](../../hardware/conexiones-esp32-s3.md) |
| `QTR_A_CTRL` | **42** | Mismo GPIO que `QTR_EMITTER_CTRL` — comparte pin físico con `QTR_B_CTRL`, ver abajo |
| `QTR_B_SENSOR` | **2** | QTRX-HD-01A, modo analógico — mismo GPIO que `QTR_RIGHT_OUT` |
| `QTR_B_CTRL` | **42** | Mismo pin que `QTR_A_CTRL`: las 2 luces IR (una por sensor) comparten un solo cable de control, así que se encienden y apagan juntas |
| `GENERIC_IR` | **14** | |

Los dos sensores bajo prueba son **el mismo modelo, en modo analógico** —
no hay ningún sensor en modo RC conectado ahora mismo. Los nombres del
código usan "A"/"B" en vez de "analog"/"rc" a propósito, para no dar a
entender un modo que no se está usando.

**Historial de este cableado, para que quede constancia:** este proyecto
pasó por GPIO35/25/4/26(→compartido) del archivo original, luego GPIO1/2/42
(intento de reutilizar los pines del diseño de vuelo), vuelta a
GPIO35/25/4/25 porque el equipo confirmó que ese era el cableado real,
GPIO35→GPIO5 porque GPIO35 no tiene ADC en el S3, y ahora de nuevo a
GPIO1/2/42, confirmado como el cableado físico actual. Si en algún momento
esto vuelve a no coincidir con la placa real, dilo — no hay drama en seguir
ajustando, pero cada vuelta cuesta una sesión de depuración.

## Tres partes

| Parte | Qué hace |
|---|---|
| `firmware/` | Sketch del ESP32 de banco. Solo responde: recibe el comando `'R'` por serial y contesta `DATA,<analog_a>,<analog_b>,<generic>`. Usa la librería `QTRSensors` de Pololu (única excepción del proyecto a "cero dependencias" — es una herramienta de banco, no el firmware de vuelo). |
| `calibrar_ir.py` | Script de Python que orquesta la prueba: pide moverse a cada punto, manda `'R'` repetidamente, y guarda todo en un `.csv` dentro de `data_logs/`. |
| `detector-negro-gris/` | Sketch aparte (no habla con `calibrar_ir.py`): lee en bucle, clasifica NEGRO/GRIS con un umbral elegido probando varias combinaciones contra los `.csv` ya capturados (no solo A o solo B a ojo), imprime por consola y enciende el LED RGB — ver su [README](detector-negro-gris/README.md). |

## Cómo usar

> Este sketch usa `Serial` en su configuración por defecto (UART0), **no**
> el USB nativo que sí usan `firmware-esp32/` y las pruebas de
> `pruebas-platformio/`. El puerto que le pasas a `--puerto` es el que te
> aparezca normal al conectar el ESP32-S3, sin nada especial que ajustar.

1. Conectar el ESP32-S3 (solo, con el sensor bajo prueba — sin el resto de periféricos) y subir el sketch:
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
surface,point,sample,analog_QTRX_A,analog_QTRX_B,generic_IR
BLACK,1,1,4095,3980,1
...
```

| Columna | Qué es |
|---|---|
| `analog_QTRX_A` | Lectura del sensor A (QTRX-HD-01A analógico), 0–4095 (ADC de 12 bits), promediada 8x por el sketch. |
| `analog_QTRX_B` | Lectura del sensor B (mismo modelo, mismo modo), 0–4095, promediada 8x. |
| `generic_IR` | Lectura digital (0/1) de un módulo IR genérico aparte, solo para comparar. |

## Por qué esto no reemplaza la calibración en vivo de las pruebas

`pruebas-platformio/01-mantente-en-cuadro/` y `02-cuadro-color-rgb/`
calibran el QTR **en cada arranque**, sobre el ESP32-S3 y a la altura real
de montaje en el chasis — eso sigue siendo lo que decide el umbral que
efectivamente usa el robot. Esta herramienta es para entender **cómo se
comporta el sensor** (negro vs. gris, sensor A vs. sensor B, con el módulo
IR genérico como referencia aparte) antes de decidir el margen del umbral,
no para generar el umbral final.

Análisis ya hecho con los primeros dos logs de esta herramienta: el negro
satura consistentemente cerca del máximo del ADC analógico (~4060 de media,
desviación ~135), pero el gris varía mucho según la altura/ángulo con que se
sostuvo el sensor a mano (~3400–3960 de media según el punto), llegando a
solaparse con el negro en el punto más alto. Por eso la calibración en vivo
del ESP32-S3 (a altura fija de montaje) es la que manda para el umbral real.
