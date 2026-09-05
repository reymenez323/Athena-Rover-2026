# Historial del trabajo de visión (cámara / Edge Impulse)

> ### ⚠️ Esto es un REGISTRO HISTÓRICO, no el estado actual
>
> Documenta cómo se llegó al modelo de banderas que usa el robot hoy: qué se
> probó, qué falló y por qué se decidió lo que se decidió. Se conserva porque
> esas razones no están en el código y se perderían.
>
> **Varias cosas que acá figuran como "pendientes" ya se resolvieron:**
>
> | Lo que dice abajo | Cómo quedó |
> |---|---|
> | "Decisión de arquitectura AÚN NO TOMADA: ¿FOMO o pipeline local?" | **FOMO, y solo FOMO.** El pipeline local (`proposals.py`, `classifier.py`, `detector.py`, `tracker.py`) nunca se entrenó y se borró del repo. |
> | "Hace falta código de integración nuevo para FOMO" | Hecho: `src/athena/ei_flag_detector.py`. |
> | "La llave no está cubierta por el dataset" | No hace falta: la resuelve el sensor de color del ESP32, nunca dependió de la cámara. |
> | "Confirmar el puerto serial real (`ttyUSB0` vs `ttyACM0`)" | Resuelto: `serial_port = "auto"` prueba los cuatro candidatos y usa el que abra. |
> | "Integrar el protocolo binario real (no el texto plano de prueba)" | Hecho, incluido `CMD_FLAG_SIGNAL` para señalizar la bandera con el LED. |
> | "Las 577 fotos y sus rutas desordenadas están en GitHub" | Borradas del repo: el entrenamiento se hace en Edge Impulse Studio, no acá. |
> | "Exportar como TensorFlow Lite (int8)" | Se exportó como **Linux (AARCH64)** — el `.eim` que vive en `raspberry-pi/models/`. |
>
> Para el estado real, ver [`../raspberry-pi/README.md`](../raspberry-pi/README.md)
> y [`../raspberry-pi/models/README.md`](../raspberry-pi/models/README.md).

Este documento resume una conversación larga de trabajo sobre el sistema de visión del robot Athena Rover 2026.

## Proyecto

- **Athena Rover 2026** — proyecto universitario, INTEC, Ingeniería Mecatrónica. Equipo: Reymildo y Montse.
- Repo: `github.com/reymenez323/Athena-Rover-2026`, rama `main`.
- Competencia "Retos del Rover H07": dos robots (rojo/azul) deben depositar una llave en la zona neutra y luego capturar la bandera del rival y llevarla a su zona segura.
  - Bandera: cilindro de 5 cm diámetro x 15 cm alto, color rojo o azul según equipo.
  - Llave: cubo ~20x20x20 mm.
  - Pista real: 170 x 83.5 cm, zona neutra central amarilla de 27x29 cm.
- Arquitectura de hardware: Raspberry Pi 4B (visión/decisión) ↔ serial ↔ ESP32-S3 (actuadores/sensores bajo nivel).
- Cámara real del robot: **Argom CAM20**, webcam USB estándar (V4L2 en Linux, no CSI), 720p@30fps con MJPG.

## Arquitectura de software actual (repo `raspberry-pi/`)

Pipeline de visión de 2 etapas, ya implementado en el repo, **entrenamiento local pendiente** (era el TODO original antes de empezar con Edge Impulse):

1. `proposals.py`: detector de color HSV (rojo/azul) que propone regiones candidatas — barato, corre en 2-4ms.
2. `classifier.py`: CNN INT8 TFLite (64x64 de entrada) que clasifica cada recorte en 4 clases: `bandera_roja`, `bandera_azul`, `llave`, `fondo`.
3. `detector.py` orquesta todo + tracking (`tracker.py`) + geometría/distancia (`geometry.py`).
4. Comunicación con el ESP32-S3 vía protocolo **binario** propio (`protocol.py`) — no texto plano.
5. Config en `config.py`: `serial_port` por defecto es `/dev/ttyACM0`, pero en la práctica el ESP32 aparece como **`/dev/ttyUSB0`** — hay que confirmar/corregir cuál usar de verdad.
6. `models/athena_cls.tflite` (el modelo local) **todavía no existe / no se ha entrenado**. Si no existe, el sistema cae a reglas geométricas de color puro (modo degradado, funcional pero menos preciso).

## Objetivo de esta conversación

Evaluar/usar **Edge Impulse** (cuenta de Montse, login con GitHub) para entrenar un detector de banderas por bounding boxes (FOMO), como posible reemplazo o complemento del pipeline local.

## Hallazgos sobre material de otros equipos

### 1. Modelo ya entrenado de un compañero de otro equipo (`.zip`)
- Es una **librería Arduino/C++** exportada de Edge Impulse (`a1125673-project-1_inferencing`), **NO un `.tflite` suelto**. Pensada para compilar dentro de un microcontrolador con cámara propia (trae ejemplos para ESP32-CAM, Nicla Vision, Portenta H7), **no es compatible directamente** con el pipeline Python/`tflite_runtime` de la Raspberry Pi.
- Modelo: **FOMO** (object detection por bounding boxes), entrada **96x96**, **2 clases**: `CILINDRO_AZUL`, `CILINDRO_ROJO`. Sin llave, sin clase de fondo explícita (FOMO no la necesita).
- ⚠️ El código trae un aviso legal de Edge Impulse: *"You may NOT use this Software unless you have an active Edge Impulse subscription..."* — **no se ha confirmado** con el compañero si aplica. Pendiente de verificar antes de construir más sobre ese artefacto específico (el dataset de fotos es un tema aparte, ver abajo).

### 2. Fotos de entrenamiento "de la mayoría de los equipos" (`.zip`)
- Carpetas: `CAZUL` (67), `CROJO` (64), `BACKGROUND+DATA` (129), `DATABASE` (317) = 577 fotos.
- `DATABASE` resultó ser un **export previo de un proyecto de Edge Impulse** (nombres con patrón `ingestion-<hash>`), y **131 de sus archivos eran duplicados exactos** de `CAZUL`/`CROJO` → total real único: **446 fotos**.
- Resolución baja (320x240, ~7-12KB), cámara tipo ESP32-CAM/espcam — **cámara distinta a la Argom real** del proyecto (riesgo de transferencia de dominio, ya discutido y parcialmente confirmado como problema real, ver resultados de Model Testing más abajo).

## Herramientas construidas (en `raspberry-pi/scripts/`, YA COMMITEADAS en `5662548`)

### `auto_label_ei.py`
- Reutiliza `athena.proposals.ProposalGenerator` (el mismo detector de color HSV que usa el robot) para generar bounding boxes candidatos por color en cada foto.
- Detecta el color real de cada foto, no confía en el nombre de carpeta — **excepto** en carpetas reconocidas como fondo (`CARPETAS_FONDO = {BACKGROUND+DATA, FONDO, BACKGROUND}`), donde **fuerza cero cajas sin importar el color detectado** (ajuste hecho tras encontrar falsos positivos reales: alguien sosteniendo el cilindro-prop sin pintar en fotos de "fondo" disparaba falsos positivos de rojo/azul por reflejos de ventana y tono de piel/pelo).
- Etiquetas usadas: `CILINDRO_ROJO`, `CILINDRO_AZUL` (mismos nombres que el modelo del compañero, por compatibilidad).
- Salida: `bounding_boxes.labels` (JSON, formato que Edge Impulse reconoce al subir junto con las fotos), carpeta `previews/` con las cajas dibujadas, `revisar_manualmente.txt` (fotos sin detección en carpetas donde SÍ se esperaba objeto).
- Uso: `python3 scripts/auto_label_ei.py --input <carpeta> --output <carpeta>`

### `capture_webcam_laptop.py`
- Captura fotos con cualquier webcam USB (laptop o Raspberry Pi — no depende del backend V4L2 explícito como `capture_dataset.py`, que sí es específico de Linux/RPi).
- Modo prueba (confirmar qué índice de cámara es cuál, antes de capturar en serio): `python3 capture_webcam_laptop.py --probar`
- Modo captura: la clase se elige y se cambia **en vivo con teclas**, sin reiniciar el script:
  - `1` = fondo, `2` = cilindro_azul, `3` = cilindro_rojo
  - `ESPACIO` = guardar foto de la clase activa
  - `a` = alterna captura automática
  - `q` = salir (muestra resumen de cuántas fotos por clase)

## Proyecto en Edge Impulse (cuenta de Montse)

- Nombre: `athena-rover-banderas` (sugerido).
- Tipo: **Images → Object Detection**.
- **Target device: Raspberry Pi 4** (Cortex-A, 1.8GHz) — coincide con el hardware real.
- Impulse:
  - Image width/height: **120x120** — ⚠️ IMPORTANTE: **FOMO exige entradas cuadradas**, tuvimos un error real (`Exception: Only square inputs are supported`) al intentar 160x120, no cuadrado.
  - Resize mode: **Squash** (no "Fit shortest axis" — con fotos 4:3 esa opción recorta bordes izquierdo/derecho, y "Squash" solo achica sin perder contenido, aunque distorsiona un poco la forma).
  - Color depth: **RGB** (crítico — nunca Grayscale, porque el color es la única señal que distingue equipo rojo de azul).
- Learning block: **Object Detection → FOMO (MobileNetV2 0.35)**.
- Training: 60 ciclos, LR 0.001, CPU, data augmentation activado.

## Primera tanda de datos subida (446 fotos deduplicadas del batch de otros equipos)

- Montse identificó 34 falsos positivos en las previews, se removieron del `bounding_boxes.labels` antes de subir.
- Upload: 445/446 exitoso (1 falló por hash de contenido duplicado — Edge Impulse lo descartó solo, sin pérdida real de dato único).
- Split: 355 training / 90 test (82%/18%).
- Nota de UX de Edge Impulse: la pestaña "Labeling queue" mostraba (0) aunque había fotos sin caja, porque nuestro `bounding_boxes.labels` siempre incluye una entrada (aunque vacía) por foto, y Edge Impulse interpreta eso como "ya revisado sin objeto", no como "pendiente". Hubo que usar el filtro de "sin etiqueta" en la vista **Dataset** en vez de la cola de etiquetado para completar la revisión manual.
- Montse terminó de revisar/clasificar todo manualmente.

## Primer entrenamiento (445 fotos, todas de origen "espcam")

- F1 validación: **90.4%** (Background F1 1.00, Azul F1 0.88, Rojo F1 0.92).
- **0% de confusión cruzada entre rojo y azul** — el dato de seguridad más importante para la competencia, se mantuvo en todas las pruebas posteriores también.
- Errores: bandera confundida con fondo (11.5% azul, 13.6% rojo), nunca color por color.
- On-device: 5ms inferencia, 182KB RAM, 81KB flash (EON Compiler).

### Model Testing (90 fotos de test nunca vistas)
- Accuracy general: **65.56%** (unoptimized float32) — caída fuerte vs validación.
- Precision no-fondo: **0.61** (vs 0.94 en validación) — el modelo daba falsos positivos sobre fondos nuevos que no había visto.
- Recall no-fondo: 0.88. F1 no-fondo: 0.72.
- Sigue en 0% confusión rojo↔azul en test también.
- Diagnóstico confirmado por la matriz de confusión a nivel de celda: el problema es la clase **fondo**, que no generalizaba a escenas nuevas — root cause: el dataset de fondo original (129 fotos, cámara espcam) no tenía suficiente variedad.

## Acción tomada para mejorar (último paso confirmado)

- Se capturaron **167 fotos NUEVAS directo con la Argom CAM20 real**, vía Raspberry Pi (usando `capture_webcam_laptop.py`, corrido ahí guardado como `capture.py` vía `nano`+SSH): 60 cilindro_azul, 57 cilindro_rojo, 50 fondo.
- Al pre-etiquetar se descubrió el problema de falsos positivos en "fondo" (alguien sosteniendo el cilindro-prop sin pintar, ventana oscura detectada como azul, piel/pelo como rojo) → se corrigió `auto_label_ei.py` para forzar cero cajas en carpetas de fondo reconocidas.
- **Pendiente sin resolver**: se encontraron 15 fotos con nombre `fondo_...` guardadas físicamente dentro de la carpeta `cilindro_azul` (no se confirmó con Montse si es error de captura real o solo desorden de archivos).
- Montse subió las 167 fotos nuevas a Edge Impulse (se suman a las 445 anteriores → dataset combinado ~612 fotos), regeneró features (Image block sigue en 120x120/Squash/RGB) y **acaba de reentrenar** el modelo con el dataset combinado.

## ESTADO ACTUAL — qué falta hacer (continuar desde aquí)

1. **Correr de nuevo "Model testing" → "Classify all"** sobre el modelo reentrenado, para ver si mejoró el problema de falsos positivos en fondo (comparar contra el 65.56% / F1 no-fondo 0.72 anterior). Esto es lo inmediato siguiente.
2. Confirmar con Montse sobre las 15 fotos `fondo_...` que quedaron en la carpeta `cilindro_azul` — ¿son fondo mal guardado o azul real?
3. **Decisión de arquitectura AÚN NO TOMADA**: ¿reemplazar todo el pipeline local con el detector FOMO de Edge Impulse, o un híbrido (FOMO solo para banderas, pipeline local color+CNN para llave y fondo)? La llave nunca formó parte del dataset de Edge Impulse — sigue sin cubrirse por ese lado.
4. Cuando el modelo esté listo: exportar desde Edge Impulse como **"TensorFlow Lite" (int8)**, NO la librería Arduino/C++ (esa es para microcontroladores, no sirve para el pipeline Python de la Raspberry Pi). Aún no se ha hecho ningún export.
5. Si se decide usar FOMO en producción: hace falta **código de integración nuevo**. El `classifier.py` actual del repo espera un modelo de CLASIFICACIÓN de recortes (recorte→clase), no de DETECCIÓN por bounding boxes de grilla (que es como sale FOMO). Habría que escribir un módulo nuevo que consuma la salida de FOMO y produzca objetos `Detection`/`BBox` compatibles con `types.py`.
6. Confirmar puerto serial real del ESP32 (`/dev/ttyUSB0` visto en la práctica vs `/dev/ttyACM0` que trae `config.py` por defecto) y corregir el config si hace falta.
7. Integrar el protocolo binario real del repo (no el texto plano `RED`/`BLUE`/`OFF` que se usó solo como prueba inicial de comunicación con el LED).

## ⚠️ Hallazgo de estado del repositorio (importante, descubierto al armar este resumen)

Fuera de esta conversación, en otra terminal, **Reymildo ya comiteó y empujó a `origin/main`** (commit `5662548 "Add training photos and Edge Impulse auto-label output"`) las **577 fotos de entrenamiento** (de otros equipos) junto con las salidas de `auto_label_ei.py`, en rutas algo desordenadas:
- `raspberry-pi/scripts/FOTOS ENTRENAMIENTO/FOTOS ENTRENAMIENTO/...`
- `raspberry-pi/scripts/data/ei_labels/...`

Esto ya está público en GitHub. Vale la pena revisar si:
- Se quiere reorganizar esas rutas (no siguen la convención `data/raw/<clase>/` que ya usa el resto del repo).
- Hay algún problema de compartir fotos de otros equipos públicamente, dado el aviso de licencia de Edge Impulse mencionado arriba (aunque ese aviso specific era sobre el modelo/librería exportada, no necesariamente sobre las fotos crudas — no está 100% claro, vale confirmarlo).

Los scripts `auto_label_ei.py` y `capture_webcam_laptop.py` también quedaron commiteados — eso sí está bien y es lo esperado.

## Preferencias de Montse (para quien continúe)

- Prefiere que se le explique **paso a paso, con calma**, confirmando cada pantalla/resultado antes de seguir (pide capturas de pantalla en cada paso y las manda de vuelta).
- Aprecia que se le avise de errores de forma directa y honesta (ej.: cuando se le recomendó un tamaño de imagen no-cuadrado que luego resultó incompatible con FOMO, se le explicó el error claramente en vez de disimularlo).
- En un momento puntual de la conversación pidió que se le hablara "como dominicano" — fue una petición **explícitamente escopeada a esa conversación puntual**, no una preferencia permanente.
- Idioma de trabajo: español.

## Archivos entregados a Montse durante la sesión (vía chat, no están todos en el repo)
- `ei_labels_out.zip` — primera tanda de previews + `revisar_manualmente.txt` + labels crudo, antes de deduplicar.
- `subir_a_edge_impulse.zip` — 446 fotos deduplicadas + labels corregido (primera tanda, la que se subió).
- `capture_webcam_laptop.py` — script de captura (dos versiones entregadas, la vigente es la de selección de clase en vivo).
- `subir_pi.zip` y `fotos_pi_previews.zip` — segunda tanda, 167 fotos de la Argom real (la que se acaba de subir y reentrenar).
