# Pruebas PlatformIO

Proyectos PlatformIO pequeños y **independientes entre sí y del firmware
principal** (`firmware-esp32/`). Cada uno valida un solo comportamiento con
hardware real antes de integrarlo a las 7 tareas de FreeRTOS del firmware
final. Reutilizan los pines de [`../hardware/conexiones-esp32-s3.md`](../hardware/conexiones-esp32-s3.md)
para poder cablearse directo sobre el chasis ya armado.

Cada carpeta es su propio proyecto PlatformIO (`platformio.ini` + `src/`):
se abre y se compila por separado, no como sub-proyecto de otro.

## Pruebas

| Carpeta | Qué valida |
|---|---|
| [`01-mantente-en-cuadro/`](01-mantente-en-cuadro/) | Los 2 QTR delanteros + los 2 L298N: detectar el borde de cinta negra y quedarse dentro del cuadrado. |
| [`02-cuadro-color-rgb/`](02-cuadro-color-rgb/) | Lo anterior + los 2 TCS34725 clasificando color (umbrales, sin KNN) + un LED RGB mostrando el color del sensor delantero. |
| [`03-motores-adelante/`](03-motores-adelante/) | Solo los 2 L298N: los 4 motores adelante a velocidad máxima constante, sin sensores ni lógica. Prueba mínima de cableado y de subida. |

## Ideas para próximas pruebas

Sin empezar todavía — usar el mismo patrón (una carpeta, un `platformio.ini`,
un `README.md` con qué se está validando y por qué):

- Un solo servo por el PCA9685, para calibrar los ángulos del gripper
  desacoplado del mecanismo.
