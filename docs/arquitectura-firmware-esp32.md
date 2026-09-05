# Arquitectura del firmware ESP32-S3

> La referencia detallada y actualizada vive junto al código, en
> [`firmware-esp32/README.md`](../firmware-esp32/README.md). Este documento
> explica el **porqué** de las decisiones de diseño.

## El requisito

El equipo pidió explícitamente: *"cada tarea independiente, que puedan fallar
sin afectar a las otras directamente"*, y *"que todas las tareas estén en el
main"*, con librerías externas solo si ahorran muchísimo trabajo.

Eso se cumple así:

- **Un solo archivo**, `firmware-esp32/src/main.cpp`, con las 8 tareas dentro.
- **Cero librerías externas.** Los drivers del PCA9685 y del TCS34725 están
  escritos a mano (unas 40 y 60 líneas). Traer librerías para eso no habría
  ahorrado trabajo real y sí habría añadido dependencias que mantener.

## Por qué colas y no un estado global compartido

La alternativa habitual es una estructura global con el estado del robot,
protegida por un mutex. Se descartó por una razón concreta: **un mutex es un
punto de acoplamiento**. Si la tarea del sensor de color se cuelga sosteniendo
el mutex, la tarea de los motores se bloquea esperándolo — que es exactamente
el fallo en cascada que se quería evitar.

Con colas, cada tarea es dueña de sus datos. Nadie espera a nadie. Si una tarea
muere, sus colas simplemente dejan de llenarse y las demás siguen su ritmo.

## Por qué el watchdog no reinicia el ESP32

Lo fácil sería llamar a `esp_restart()` al detectar una tarea colgada. No se
hace, porque en plena ronda un reinicio significa: motores parados, gripper
suelto (¡se cae la bandera!), LED de equipo apagado y varios segundos de
arranque. Perder la ronda, básicamente.

En vez de eso, el supervisor **reporta** el fallo a la Raspberry Pi, que es
donde vive la lógica de decisión. La Pi puede entonces dejar de confiar en ese
subsistema y seguir compitiendo con lo que quede. Un robot con el sensor de
color muerto todavía puede ganar; un robot reiniciándose, no.

## Por qué el failsafe de los motores es local

`MotorTask` frena sola si pasa 500 ms sin recibir un comando válido. Podría
haberse puesto esa lógica en `SerialTask`, que es quien sabe si llegan datos.

No se hizo justamente por eso: si `SerialTask` es la que se cuelga, sería la
responsable de detectar su propio fallo. Poniendo el temporizador dentro de
`MotorTask`, el robot frena aunque el problema esté en la tarea de
comunicación, en el cable USB, o en la Raspberry Pi.

## Por qué el protocolo se serializa byte a byte

La primera versión mandaba `structs` con `memcpy` y `sizeof`. Es lo cómodo y es
un error: el compilador inserta *padding* invisible entre campos según sus
reglas de alineación, y ese padding no tiene por qué coincidir con lo que Python
asuma del otro lado. El síntoma sería telemetría con valores absurdos, en un
sitio donde nadie mira.

Ahora cada campo se escribe explícitamente en little-endian y los booleanos
viajan empacados en bits. El contrato está fijo, documentado, y **verificado por
un test** que lee el `main.cpp` real y lo compara con el Python.

## Prioridades y núcleos

| | Núcleo 0 | Núcleo 1 |
|---|---|---|
| | Supervisor, sensores, LED | Motores, comunicación, gripper |

El criterio: lo que afecta al movimiento del robot va junto en el núcleo 1, para
que ningún sensor lento pueda robarle tiempo al lazo de control. El supervisor
va en el 0 con la prioridad más alta del sistema, para que siga vigilando
aunque el núcleo 1 esté saturado.
