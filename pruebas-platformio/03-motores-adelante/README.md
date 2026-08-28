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

No cambies el cableado físico. En `setup()`, para ese motor en particular,
llama a `MotorReverseFull(...)` en vez de `MotorForwardFull(...)` — ya está
en `src/main.cpp`, es la misma función con `IN1`/`IN2` invertidos.

> Ya aplicado: con el cableado físico actual, la rueda trasera izquierda es
> la que quedó conectada a OUT1/OUT2 del L298N izquierdo — eso es `kMotorFL`
> en el código (IN1/IN2/ENA), **no** `kMotorRL`, pese al nombre. Es esa rueda
> la que gira al revés, así que en `setup()` usa `MotorReverseFull(kMotorFL)`.
> Si en algún momento intercambias los 2 cables de ese motor en el borne
> físico, vuelve a poner `MotorForwardFull(kMotorFL)` aquí.
>
> **Ojo con el nombre de la constante:** `kMotorFL`/`kMotorRL` describen qué
> par de pines del L298N usan (IN1/IN2 vs. IN3/IN4), no necesariamente qué
> rueda física es delantera o trasera en este chasis — eso depende de a qué
> borne conectaron cada motor. No asumas front/rear por el nombre; verifica
> contra el cableado real.
> Esta corrección es **local a esta prueba** — si más adelante el firmware
> principal (`firmware-esp32/`) o cualquier otra prueba también manda
> `MotorMode::DRIVE` a ese mismo motor, va a necesitar el mismo ajuste, ya
> que ninguno de los dos comparte código con este sketch.
