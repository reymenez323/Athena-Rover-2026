# Modelos

Aquí vive `athena_cls.tflite`, el clasificador cuantizado a INT8 que corre en
la Raspberry Pi. **No se versiona en git** (lo ignora `.gitignore`): se
regenera con `python3 scripts/train_classifier.py`.

## Por qué INT8 y no float

En una Raspberry Pi 4B no hay GPU utilizable para redes neuronales, así que
todo cae sobre los cuatro núcleos ARM. Un modelo cuantizado a enteros de 8
bits ocupa 4 veces menos memoria y corre entre 2 y 3 veces más rápido que el
mismo modelo en float32, porque el delegado XNNPACK usa instrucciones SIMD
enteras del procesador. La pérdida de precisión en un problema de 4 clases
tan separadas como éste es despreciable.

## Si no hay modelo

`detector.py`/`classifier.py` (el camino que usa este `.tflite`) ya **no
corren dentro de `run_rover.py`** -- ver `athena_ei_banderas.eim` abajo, que
lo reemplazó para encontrar la bandera. Este clasificador local queda en el
repo sin usarse por ahora (no se borró por si se retoma más adelante). Si en
algún momento vuelve a conectarse a algo, su modo degradado sigue intacto: sin
modelo, `detector.py` cae a las reglas geométricas y de color puras.

## `athena_ei_banderas.eim`

Modelo de detección de banderas (FOMO, 2 clases: `CILINDRO_ROJO`/
`CILINDRO_AZUL`) entrenado en Edge Impulse con fotos tomadas con la Argom
CAM20 real del robot. **Tampoco se versiona en git** (pesa ~13 MB). **Es
obligatorio para `scripts/run_rover.py`** -- sin él, el robot no arranca (falla
rápido y claro al no encontrarlo, a propósito, en vez de arrancar a medias sin
poder ver banderas). También lo usa `scripts/run_flag_tracker_ei.py`, que
ahora es solo una herramienta aparte para calibrar a mano la zona muerta y la
ganancia de giro con `--ver`, sin la secuencia completa de la misión.

La llave nunca dependió de la cámara -- `decision.py` la maneja enteramente
con el sensor de color del ESP32 (`ColorTelemetry`), así que no hace falta un
modelo aparte para eso.

Para regenerarlo: reentrenar en Edge Impulse Studio (proyecto
`athena-rover-banderas`) y exportar con **Deployment target: "Linux
(AARCH64)"** (no "TensorFlow Lite" -- esa opción no existe para modelos de
Object Detection en este proyecto -- ni ningún target de Arduino/C++, que
generan código para microcontrolador y no sirven para el pipeline en Python
de la Raspberry Pi). El `.eim` exportado necesita permiso de ejecución en
Linux: `chmod +x models/athena_ei_banderas.eim`.
