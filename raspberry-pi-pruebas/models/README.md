# Modelos

## `athena_ei_banderas.eim`

El detector de banderas del robot. Es lo único que la cámara tiene que
reconocer, y **es obligatorio**: sin él, `run_rover.py` no arranca (falla
rápido y claro al no encontrarlo, a propósito, en vez de arrancar a medias sin
poder ver banderas).

| | |
|---|---|
| Arquitectura | FOMO (MobileNetV2 0.35), detección por *bounding boxes* |
| Entrada | 120×120, RGB, modo *Squash* |
| Clases | `CILINDRO_ROJO`, `CILINDRO_AZUL` |
| Entrenado en | Edge Impulse Studio, proyecto `athena-rover-banderas` (cuenta de Montse) |
| Dataset | ~612 fotos: 445 de otros equipos (cámara espcam) + 167 tomadas con la cámara real del robot |
| Export | Deployment target **"Linux (AARCH64)"** |

> Las 167 fotos buenas se tomaron con la cámara que el robot lleva de verdad,
> así que ese trozo del dataset sí es del dominio real. La cámara aparece con
> dos nombres —**Argom CAM20** y **Xtrike Me XPC-01**— pero es la misma:
> mismo fabricante, distinto vendedor.

**Sí se versiona en git**, aunque pesa 13 MB. Es la excepción del
`.gitignore`, y es deliberado: no se puede regenerar sin la cuenta de Edge
Impulse de Montse, así que tenerlo en el repo es lo que hace que un clon nuevo
pueda competir el mismo día.

En la Pi necesita permiso de ejecución — es un binario, no un archivo de datos:

```bash
chmod +x models/athena_ei_banderas.eim
```

## La llave no tiene modelo, y no lo necesita

Nunca formó parte del dataset. El robot arranca con la llave ya en la pinza y
la suelta cuando el sensor de color del ESP32 ve el amarillo de la zona neutra
(ver `decision.py`). Resolverlo por visión sería hacer por el camino difícil
algo que ya está resuelto por el fácil.

## Para regenerarlo

Reentrenar en Edge Impulse Studio y exportar con **Deployment target: "Linux
(AARCH64)"**.

No sirven los otros targets: "TensorFlow Lite" no existe como opción para
modelos de Object Detection en este proyecto, y los targets de Arduino/C++
generan código para microcontrolador, no para el pipeline en Python de la Pi.

## Lo que se sabe del rendimiento

Del último ciclo de entrenamiento documentado (ver
[`docs/handoff-vision-edge-impulse.md`](../../docs/handoff-vision-edge-impulse.md)):

- **0 % de confusión entre rojo y azul**, en validación y en test. Es el dato
  más importante para la competencia: el robot nunca agarró la bandera de su
  propio equipo por confundir el color.
- El punto débil es la clase **fondo**: daba falsos positivos sobre escenas
  nuevas que no había visto. Por eso se capturaron las 167 fotos adicionales
  con la cámara real.
- **Pendiente:** correr "Model testing → Classify all" sobre el modelo
  reentrenado y comparar contra el 65.56 % anterior. Ese número todavía no se
  midió.
