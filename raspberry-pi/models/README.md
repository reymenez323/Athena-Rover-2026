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

El sistema **funciona igual**, en modo degradado: `detector.py` cae a las
reglas geométricas y de color puras. Detecta menos fino y se traga más falsos
positivos, pero el robot se mueve. Eso permite probar mecánica y control desde
el primer día, sin esperar a tener el dataset listo.
