# Prueba 03 — Motores adelante

La prueba más simple del proyecto: enciende los 4 motores hacia adelante, a
velocidad máxima y constante, y no hace nada más. Sin sensores, sin
calibración, sin máquina de estados.

Usa los mismos pines de motores documentados en
[`../../hardware/conexiones-esp32-s3.md`](../../hardware/conexiones-esp32-s3.md#motores--2-l298n),
así que corre tal cual sobre el chasis ya cableado (solo necesita los 2
L298N — no requiere QTR, TCS34725, PCA9685 ni LED RGB para esta prueba).

## ⚠️ Antes de subir

Pon el robot sobre un soporte con las **4 ruedas al aire**, o en el piso con
espacio libre por delante. El robot arranca en cuanto termina `setup()`, sin
espera ni confirmación — no hay forma de detenerlo por software una vez
subido, solo desconectando la batería de motores o reseteando el ESP32.

## Compilar y subir

```bash
pio run -t upload -t monitor
```

El monitor usa el puerto **UART** del DevKit (no el "USB" nativo), igual que
el resto de las pruebas — ver `platformio.ini`.

## Para qué sirve

- **Confirmar el cableado de motores** antes de sumarle sensores o lógica:
  jumpers ENA/ENB quitados, sentido de giro correcto en las 4 ruedas.
- **Prueba mínima de subida.** Si este proyecto compila, sube y el robot
  avanza, cualquier problema que hayas tenido con PlatformIO/esptool (por
  ejemplo un `Write timeout`) era del puerto, el cable o el modo de arranque
  del ESP32-S3 — no del proyecto en sí.

## Si alguna rueda gira al revés

No cambies el cableado físico ni agregues una función aparte. En la
definición de ese motor (arriba de `setup()`), intercambia el orden de los
dos pines `IN1`/`IN2` que le pasas al struct `Motor` — `MotorForwardFull()`
queda igual para los 4 motores, sin ramas ni casos especiales.

> Ya aplicado: con el cableado físico actual, la rueda trasera izquierda es
> la que quedó conectada a OUT1/OUT2 del L298N izquierdo — eso es `kMotorFL`
> en el código (IN1/IN2/ENA), **no** `kMotorRL`, pese al nombre. Es esa rueda
> la que gira al revés, así que su definición queda:
> ```cpp
> constexpr Motor kMotorFL = {Pins::L298N_L_IN2, Pins::L298N_L_IN1, Pins::L298N_L_ENA, 0};
> ```
> (nota el orden `IN2, IN1` en vez de `IN1, IN2`). Si en algún momento
> intercambias los 2 cables de ese motor en el borne físico, vuelve a poner
> `IN1, IN2` en orden.
>
> **Ojo con el nombre de la constante:** `kMotorFL`/`kMotorRL` describen qué
> par de pines del L298N usan (IN1/IN2 vs. IN3/IN4), no necesariamente qué
> rueda física es delantera o trasera en este chasis — eso depende de a qué
> borne conectaron cada motor. No asumas front/rear por el nombre; verifica
> contra el cableado real.
>
> Esta corrección está aplicada por separado en cada proyecto que controla
> motores (`01-mantente-en-cuadro`, `02-cuadro-color-rgb`, `firmware-esp32/`)
> porque ninguno comparte código con este sketch — si agregan un proyecto
> nuevo que controle motores, hay que repetirla ahí también.
