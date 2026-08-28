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
| [`01-mantente-en-cuadro/`](01-mantente-en-cuadro/) | El QTR derecho (solo ese, ver su README) + los 2 L298N: detectar el borde de cinta negra, retroceder y girar 180° para quedarse dentro del cuadrado. |
| [`02-cuadro-color-rgb/`](02-cuadro-color-rgb/) | Lo anterior + los 2 TCS34725 clasificando color (umbrales, sin KNN) + un LED RGB mostrando el color del sensor delantero. |
| [`03-motores-adelante/`](03-motores-adelante/) | Solo los 2 L298N: los 4 motores adelante a velocidad máxima constante, sin sensores ni lógica. Prueba mínima de cableado y de subida. |
| [`04-servos-a-cero/`](04-servos-a-cero/) | Solo el PCA9685: manda los 2 servos del gripper (pinza y elevación) a 0°, referencia mecánica fija para montar los cuernos del servo. |
| [`05-evitador-linea/`](05-evitador-linea/) | Los 2 QTR (con prioridad) + los 2 TCS34725 (de respaldo, solo para NEGRO, umbrales calibrados con datos reales — ver `calibracion-color/detector-tcs/`) + los 2 L298N: evitador reactivo que gira sobre su eje sin temporizadores fijos, siempre dentro del cuadrado. |

## Ideas para próximas pruebas

Sin empezar todavía — usar el mismo patrón (una carpeta, un `platformio.ini`,
un `README.md` con qué se está validando y por qué):

- Los demás servos/mecanismos del gripper que hagan falta más allá de la
  posición 0° de `04-servos-a-cero/`.
