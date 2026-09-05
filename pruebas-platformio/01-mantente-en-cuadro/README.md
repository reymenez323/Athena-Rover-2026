# Prueba 01 — Mantente en el cuadro

Robot que avanza dentro del cuadrado delimitado por cinta negra (fondo gris
de la pista), y en cuanto el sensor de reflectancia **derecho** confirma el
borde (fondo gris -> cinta negra), **retrocede, gira 180° sobre su propio
eje y retoma la marcha** — se queda rebotando dentro del cuadro en vez de
atascarse contra el borde.

**Solo se usa el sensor derecho.** El izquierdo no se lee en ningún punto de
este archivo: nunca decidió nada, así que se le quitó también la calibración
y la telemetría — ver el encabezado de `src/main.cpp`.

> Ver también [`05-evitador-linea/`](../05-evitador-linea/): un diseño
> distinto para el mismo problema, reactivo (sin temporizadores fijos) y
> con un segundo sensor (color, clasificado por K-NN) además del QTR.

Es un sketch de banco, independiente del firmware principal
(`firmware-esp32/`): sirve para validar el comportamiento de borde con
hardware real antes de integrarlo a las 8 tareas de FreeRTOS. Usa los mismos
pines documentados en [`../../hardware/conexiones-esp32-s3.md`](../../hardware/conexiones-esp32-s3.md),
así que corre tal cual sobre el chasis ya cableado (solo necesita el QTR
derecho y los 2 L298N — no requiere PCA9685 ni TCS34725 para esta prueba).

## Compilar y subir

```bash
pio run -t upload -t monitor
```

El monitor usa el puerto **UART** del DevKit (no el "USB" nativo), igual que
el firmware principal — ver `platformio.ini`.

## Motor invertido

Con el cableado físico actual, la rueda trasera izquierda —la que quedó
conectada a OUT1/OUT2 del L298N izquierdo, que en el código es `kMotorFL`—
gira al revés respecto a las otras tres. Es un hecho fijo del cableado, no
depende de la velocidad, así que se resuelve **una sola vez, en la
definición de `kMotorFL`**, intercambiando el orden de `L298N_L_IN1` y
`L298N_L_IN2`:

```cpp
constexpr Motor kMotorFL = {Pins::L298N_L_IN2, Pins::L298N_L_IN1, Pins::L298N_L_ENA, 0};
```

`MotorApply()` es idéntica para los 4 motores — no hay ninguna rama ni flag
de inversión, solo cuál GPIO hace de "in1" y cuál de "in2" para este motor
en particular. Si en algún momento recableas ese motor para que coincida
con los otros tres, vuelve a poner `L298N_L_IN1, L298N_L_IN2` en orden.

## Cómo se calibra el umbral

Al encender, el LED rojo de equipo parpadea durante 3 segundos: es la señal
para pasar el robot **a mano** sobre la cinta negra y el piso gris varias
veces, cubriendo el sensor derecho. El sketch se queda con el mínimo y el
máximo que vio y usa el punto medio como umbral.

Por qué calibra en cada arranque en vez de usar un número fijo: ver el
análisis completo en el encabezado de `src/main.cpp` y en
[`../../calibracion/reflectancia/README.md`](../../calibracion/reflectancia/README.md) — en
resumen, el contraste NEGRO/GRIS medido a mano (sin altura fija) varía
demasiado sesión a sesión como para confiar en una constante copiada de un
log puntual.

Si en esos 3 segundos el sensor no ve suficiente contraste, se avisa por
consola y usa el umbral de reserva `FALLBACK_THRESHOLD` (2910 al momento de
escribir esto — sacado del sensor derecho/B, ver
`../../calibracion/reflectancia/detector-negro-gris/src/main.cpp`).

## Parámetros para ajustar

Todos están al principio de `src/main.cpp`, con comentarios:

| Constante | Qué controla |
|---|---|
| `kForwardSpeed` | Velocidad de avance (0–100 %) |
| `kReverseSpeed` | Velocidad de reversa (0–100 %, con signo negativo) |
| `kTurnSpeed` | Duty de cada rueda durante el giro en el sitio — va al máximo (100) a propósito, ver "El giro" abajo |
| `kReverseMs` / `kTurn180Ms` | Duración de cada maniobra — de LAZO ABIERTO (sin encoders ni giroscopio), hay que ajustarlas a ojo sobre el chasis real, ver la nota en el código |
| `kConfirmacionesNecesarias` | Cuántas lecturas seguidas por encima/debajo del umbral hacen falta para confiar en la detección (ver "Margen contra lecturas erróneas" abajo) |
| `MIN_USABLE_SPAN` | Contraste mínimo para aceptar la calibración en vivo |
| `FALLBACK_THRESHOLD` | Umbral de reserva si la calibración no sirvió |

## El giro

Pivote en el sitio: una rueda adelante, la otra atrás (`DriveSides(kTurnSpeed, -kTurnSpeed)`),
nunca un giro tipo auto con un solo lado más rápido que el otro. Un pivote
con 4 ruedas motrices necesita mucho más torque que avanzar derecho (las 4
raspan contra el piso en vez de rodar limpio) — por eso `kTurnSpeed` está al
máximo duty (100), no a la misma velocidad que avanzar. Si aun así casi no
gira, ya no es un ajuste de software: revisar tracción, que la batería de
motores no caiga de voltaje bajo esa carga, o algún roce mecánico.

`kReverseMs`/`kTurn180Ms` son de lazo abierto — no hay forma de que el
sketch sepa cuántos grados giró de verdad, así que si se pasa o se queda
corto, se ajustan proporcionalmente (ver el comentario junto a esas
constantes en el código).

## Margen contra lecturas erróneas

El ADC del QTR tiene ruido puntual, sobre todo cerca del umbral — una sola
lectura de más no debería bastar para arrancar la maniobra. Por eso una
lectura cruda por encima del umbral no dispara nada por sí sola: el sensor
derecho lleva su propio contador (`Confirmar()` en `src/main.cpp`) que solo
sube mientras la lectura siga marcando negro, y se reinicia a 0 en cuanto
aparece una sola lectura de gris. Recién cuando el contador llega a
`kConfirmacionesNecesarias` (4, ~20 ms de negro sostenido a los ~5 ms por
vuelta de `loop()`) se trata como una detección real. El conteo se reinicia
también al volver a `FORWARD` tras el giro, para exigir una detección
fresca (ver el comentario en el código). La telemetría por consola muestra
el estado (`FORWARD`/`REVERSING`/`TURNING`) y el conteo de confirmación.

## Qué mirar si no funciona

- Si el robot no avanza derecho: revisar el sentido de giro de las ruedas en
  `MotorApply` — debería comportarse igual que en
  [`03-motores-adelante`](../03-motores-adelante/), que ya tiene la
  dirección de `kMotorFL` corregida.
- Si nunca detecta el borde (se sale del cuadro): abrir el monitor y ver el
  valor crudo del sensor derecho, impreso cada 200 ms; comparar con el
  umbral calibrado que se imprime al final de `RunCalibration()`.
- Si nunca calibra bien (se detiene incluso sobre gris, o nunca detecta el
  negro): la calibración no vio suficiente contraste — repetir el arranque
  pasando el sensor de forma más deliberada sobre ambas superficies durante
  el parpadeo del LED rojo.
- Si retrocede en ráfagas o parece trabarse al girar: casi seguro el giro
  no está completando los 180° (ver "El giro" arriba) — el robot vuelve a
  `FORWARD` todavía cerca del borde y dispara otra reversa casi de
  inmediato. No es un bug de la máquina de estados, es la maniobra de giro
  que no está funcionando.
- Al empezar a girar se enciende el LED azul; se apaga al volver a avanzar.
