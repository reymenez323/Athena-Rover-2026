# Calibración de color

Herramienta de banco para caracterizar los sensores **TCS34725** sobre
distintas superficies, **antes** de fijar los umbrales de `ClassifyColor()`
en `firmware-esp32/src/main.cpp`. No es parte del firmware de vuelo ni de
`pruebas-platformio/`.

## Por ahora, solo el sensor DELANTERO

El robot tiene 2 TCS34725 (delantero y trasero), pero la CAPTURA de
calibración (`firmware/` + `calibrar_color.py`) se concentra en uno a la
vez — hoy, el delantero. Cuál sensor está activo se decide en **un solo
lugar**: la constante `SENSOR_ES_DELANTERO` en `firmware/src/main.cpp`.
Cambiar de sensor más adelante es tocar esa constante, volver a subir el
sketch, y listo — no hay que tocar nada más del firmware. En
`calibrar_color.py`, el flag `--sensor` es solo una ETIQUETA para el nombre
del archivo y los metadatos (no controla el firmware); cuando cambies la
constante para calibrar el trasero, pásale `--sensor TRASERO` también para
que el CSV quede bien rotulado.

(`detector-tcs/` es distinto: ese sketch lee los DOS sensores a la vez
siempre, no tiene esta limitación — ver la tabla de abajo.)

## ¿Por qué eventualmente hay que calibrar los dos por separado?

Cuando le toque el turno al trasero, **no asumas que mide igual que el
delantero** — caracterízalo con esta misma herramienta y compáralos. Acá
pesa más que con los sensores IR (ver `../calibracion-ir/README.md`): la
prueba `pruebas-platformio/01-mantente-en-cuadro/` recalibra la reflectancia
EN CADA ARRANQUE, así que un desajuste del sensor se autocorrige solo.
`ColorSensorTask` en `firmware-esp32/src/main.cpp` **no hace eso** —
`ClassifyColor()` usa umbrales fijos, hardcodeados una sola vez, sin
recalibración en vivo. Si el sensor delantero y el trasero difieren (cada
uno tiene su propio LED de iluminación, están montados en posiciones
distintas del chasis, y hay variación de fábrica normal entre dos unidades
TCS34725 aunque sean el mismo modelo), un umbral compartido puede quedar
bien calibrado para uno y sistemáticamente mal para el otro — y nada lo
corrige solo durante la competencia.

Cuando tengas los CSV de ambos sensores para el mismo color, la pregunta se
responde con evidencia: si las distribuciones normalizadas de delantero y
trasero se solapan razonablemente, un umbral compartido alcanza; si no,
hacen falta dos juegos de umbrales (uno por sensor) en `ClassifyColor()`.

## Mismo ESP32-S3, pero solo con los sensores conectados

El equipo usa un único microcontrolador en todo el proyecto — el sketch de
esta carpeta corre sobre el mismo ESP32-S3, no una placa aparte. Lo que sí
es distinto es el contexto: se sube solo, con los DOS TCS34725 conectados y
**nada más** (sin motores, sin PCA9685, sin QTR) — los dos buses se
inicializan siempre aunque solo se lea uno, precisamente para que cambiar
de sensor no requiera recablear nada.

| Señal | GPIO | Nota |
|---|:---:|---|
| I2C0 SDA (delantero) | **8** | Mismo bus que el PCA9685 en el diseño de vuelo — aquí solo tiene el TCS34725 conectado |
| I2C0 SCL (delantero) | **9** | |
| I2C1 SDA (trasero) | **47** | Bus dedicado |
| I2C1 SCL (trasero) | **48** | |
| LED TCS delantero | **18** | Activo en alto, encendido fijo |
| LED TCS trasero | **21** | Activo en alto, encendido fijo |

Los dos TCS34725 tienen la MISMA dirección I2C fija (0x29) y no se puede
cambiar — por eso van en buses separados, igual que en `firmware-esp32/`.

## Cinco partes

| Parte | Qué hace |
|---|---|
| `firmware/` | Sketch del ESP32 de banco. Recibe el comando `'R'` por serial y contesta `DATA,ok,clear,r,g,b` — del sensor que indique `SENSOR_ES_DELANTERO`. El driver del TCS34725 es una copia EXACTA del que ya usa `firmware-esp32/` (mismo ATIME/GAIN) — cero librerías externas, y sobre todo, misma configuración de sensor que el robot de verdad usará. |
| `calibrar_color.py` | Script de Python que orquesta la prueba: pide sostener la muestra contra el sensor activo, manda `'R'` repetidamente, y guarda todo en un `.csv` dentro de `data_logs/` — un archivo por CADA corrida de `--superficie`, nunca se mezclan colores en un mismo CSV. |
| `data_logs/` | Los `.csv` capturados, uno por corrida — se versionan en git a propósito, son datos irremplazables. Ya tiene las 5 corridas del delantero (ver la sección de más abajo). |
| `generar_dataset_knn.py` | Convierte TODOS los CSV de `data_logs/` en un header C++ con un clasificador K-NN embebido — ver "De los CSV a un clasificador K-NN embebido" más abajo. Ya no lo usa `pruebas-platformio/05-evitador-linea/` (ver la nota en esa sección), queda disponible por si hace falta esa precisión extra en otro lado. |
| `analizar_umbrales_tcs.py` | Ajusta por descenso de coordenadas los 9 umbrales de `ClassifyColor()`/`detector-tcs` contra los CSV de `data_logs/` — mismo tipo de ejercicio que `calibracion-ir/detector-color/`, pero con 9 parámetros en vez de 1. Ya se corrió una vez (ver más abajo); volver a correrlo si cambian los datos. |
| `detector-tcs/` | Sketch aparte (no habla con `calibrar_color.py`): lee los DOS TCS34725 en vivo y clasifica cada uno por UMBRALES fijos entre AZUL/ROJO/AMARILLO/NEGRO/GRIS. **Recalibrado** con `analizar_umbrales_tcs.py` (14.3% de error contra 970 muestras reales, era 58.1% con los umbrales originales) — ver la nota completa, con matriz de confusión, en `detector-tcs/src/main.cpp`. Es la MISMA clasificación que usa `pruebas-platformio/05-evitador-linea/` (con prioridad al QTR, el color solo como respaldo) — ver ese sketch. |

## Cómo usar

> Este sketch usa `Serial` en su configuración por defecto (UART0), **no**
> el USB nativo que sí usan `firmware-esp32/` y las pruebas de
> `pruebas-platformio/`. El puerto que le pasas a `--puerto` es el que te
> aparezca normal al conectar el ESP32-S3, sin nada especial que ajustar.

1. Conectar el ESP32-S3 (solo, con los DOS TCS34725 — sin el resto de
   periféricos) y subir el sketch:
   ```bash
   cd firmware
   pio run -t upload
   ```
2. Instalar la dependencia de Python (una sola vez):
   ```bash
   pip install -r requirements.txt
   ```
3. Correr la calibración: una corrida por color, con la muestra sostenida
   contra el sensor **delantero** (el que está activo por defecto en el
   firmware). Para los 5 colores del reto:
   ```bash
   python3 calibrar_color.py --puerto COM5 --superficie AZUL     --puntos 4 --muestras 60
   python3 calibrar_color.py --puerto COM5 --superficie ROJO     --puntos 4 --muestras 60
   python3 calibrar_color.py --puerto COM5 --superficie AMARILLO --puntos 4 --muestras 60
   python3 calibrar_color.py --puerto COM5 --superficie NEGRO    --puntos 4 --muestras 60
   python3 calibrar_color.py --puerto COM5 --superficie GRIS     --puntos 4 --muestras 60
   ```
   Antes de cada punto (incluido el primero), el script cuenta regresivamente
   `--pausa-s` segundos (10 por defecto): es la ventana para acomodar la
   muestra contra el sensor.
4. El resultado queda en
   `data_logs/COLOR_<SUPERFICIE>_<SENSOR>_<fecha>_<hora>.csv` — 5 archivos,
   uno por color.

**Para calibrar el trasero más adelante:** editar `SENSOR_ES_DELANTERO =
false` en `firmware/src/main.cpp`, `pio run -t upload` de nuevo, y repetir
el paso 3 con `--sensor TRASERO` agregado a cada comando (mismos 5
colores, 5 archivos nuevos — los del delantero no se tocan).

Ver `python3 calibrar_color.py --help` para todas las opciones.

## Formato del CSV

Los metadatos de la corrida van como líneas de comentario (`#`) al inicio —
sigue siendo CSV válido, `pandas.read_csv(archivo, comment="#")` los ignora
solo:

```
# ROVER COLOR SENSOR CHARACTERIZATION
# Date: 2026-08-28 17:00:00.000000
# Surface: AZUL
# Sensor under test (label only, set by firmware): DELANTERO
# Number of points: 4
# Samples per point: 60
# Sample interval: 250 ms
# Movement time between points: 10 s
surface,sensor_under_test,point,sample,ok,clear,r,g,b
AZUL,DELANTERO,1,1,1,1450,320,410,780
...
```

| Columna | Qué es |
|---|---|
| `surface` | Color/superficie que se estaba midiendo (AZUL, ROJO, AMARILLO, NEGRO, GRIS). |
| `sensor_under_test` | DELANTERO o TRASERO — etiqueta puesta por `--sensor`, tiene que coincidir con `SENSOR_ES_DELANTERO` del firmware en esa corrida. |
| `ok` | 1 si el TCS34725 activo respondió al pedir la lectura, 0 si no (cable flojo, no conectado). Con 0, `clear/r/g/b` son 0 — ignóralos, no son "sin luz", son "no leído". |
| `clear/r/g/b` | Canales crudos del sensor (0–65535 según ganancia/tiempo de integración, mismos ATIME/GAIN que `firmware-esp32/`). `clear` es la luz total; `r/g/b` son los canales de color. |

## Qué hacer con los datos

Igual que con `../calibracion-ir/`: esto caracteriza el sensor, no genera el
umbral final por sí solo. `ClassifyColor()` en `firmware-esp32/src/main.cpp`
normaliza cada canal contra `clear` (`r/clear`, `g/clear`, `b/clear`) antes
de comparar contra umbrales — así la decisión no depende del brillo
absoluto. Con los 5 CSV del delantero puedes confirmar que se separa
limpiamente entre los 5 colores una vez normalizado por `clear`, y ajustar
los umbrales de `Umbral::` en `detector-tcs/src/main.cpp` (y luego
`ClassifyColor()` en `firmware-esp32/`) con el mismo método de
`calibracion-ir/detector-color/`: probar combinaciones contra los CSV
capturados y quedarse con la que menos errores dé.

Cuando más adelante calibres el trasero también, compara sus 5 CSV contra
los del delantero para el MISMO color — ahí es donde se responde la
pregunta de la sección de arriba: ¿un umbral compartido alcanza, o hacen
falta dos juegos separados?

## De los CSV a un clasificador K-NN embebido

> `pruebas-platformio/05-evitador-linea/` YA NO usa este clasificador —
> se cambió a los umbrales de `detector-tcs/` (sección de abajo) para que
> las dos herramientas se comporten igual. Esta sección queda como
> referencia: el script sigue funcionando, y el K-NN identifica NEGRO
> notablemente mejor que los umbrales (ver la comparación más abajo), así
> que vale la pena tenerlo a mano si en algún momento hace falta esa
> precisión extra.

`generar_dataset_knn.py` toma TODOS los CSV `COLOR_*_DELANTERO_*.csv` de
`data_logs/`, descarta las filas con `ok=0`, y escribe un header de C++
con las muestras embebidas para un clasificador K-NN escrito a mano — sin
librerías de ML. Por defecto escribe a
`pruebas-platformio/05-evitador-linea/src/dataset_color.h` (usar `--salida`
para escribir a otro lado, ya que ese sketch ya no lo incluye).

**Cuatro features por muestra, no tres.** La primera versión de este
dataset solo guardaba `r/clear`, `g/clear`, `b/clear` (el tono). Con los
datos reales de este equipo se confirmó que **`clear` (el brillo) importa**:
AMARILLO tiene una mediana de `clear` (~4511) muy por encima de todo lo
demás (~350–950 en NEGRO/AZUL/GRIS/ROJO) — descartarlo tiraba a la basura
la señal que mejor separa esa clase. El script ahora agrega `clear` como
cuarta dimensión, ESCALADO 0..1 (min-max sobre el propio dataset, ver
`kClearMin`/`kClearMax` en el header generado) para que su magnitud
(cientos a miles) no aplaste a los ratios (0..1) en la distancia del K-NN.

Correr de nuevo cada vez que cambien los CSV:

```bash
python3 generar_dataset_knn.py
```

Y volver a subir `pruebas-platformio/05-evitador-linea/` — el header
generado no se actualiza solo, y no se edita a mano (lo dice el propio
encabezado del archivo).

## De los CSV a umbrales recalibrados (`ClassifyColor()` / `detector-tcs`)

`analizar_umbrales_tcs.py` hace el mismo tipo de ejercicio que
`calibracion-ir/detector-color/` (probar combinaciones contra los datos
reales, quedarse con la que menos errores dé), pero para los 9 umbrales de
`ClassifyColor()`/`detector-tcs` en vez de 1 solo umbral — usa descenso de
coordenadas en vez de una tabla armada a mano. Resultado sobre las 970
muestras de `data_logs/`:

| | Errores | Detalle por clase |
|---|---:|---|
| Umbrales originales (sin calibrar) | 564/970 (58.1%) | AMARILLO/GRIS/AZUL casi todos mal clasificados entre sí |
| Umbrales recalibrados | 139/970 (14.3%) | AMARILLO 200/200, GRIS 196/198, ROJO 140/150, AZUL 178/222, **NEGRO 117/200** |

Ya se aplicaron a `detector-tcs/src/main.cpp` y a `ClassifyColor()` en
`firmware-esp32/src/main.cpp` (los dos deben coincidir, ver la nota en
ambos archivos). **NEGRO sigue siendo el más débil** (58.5% de acierto) —
no es un umbral mal elegido, es que su rango de `clear` se solapa mucho con
AZUL y GRIS, y una cadena de reglas con AND no separa bien esos casos. El
K-NN de la sección de arriba (mismo dataset, 4 dimensiones juntas) le
acierta bastante mejor a NEGRO — si hace falta una clasificación confiable
de NEGRO en vuelo, esa es la referencia a seguir, no seguir ajustando estos
9 umbrales.

Volver a correr si cambian los CSV:

```bash
python3 analizar_umbrales_tcs.py
```
