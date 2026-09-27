# v9-tira-sensor-trasero

Copia de [`v8-logica-completa`](../v8-logica-completa/) — **la lógica de los
12 pasos no cambia** — que agrega dos cosas físicas nuevas, cableadas el
2026-09-27:

1. **Tira WS2812B (8 LED)**, reemplaza al LED RGB de 3 canales.
2. **Sensor de color TRASERO**, recuperado detrás de un multiplexor I2C
   TCA9548A (el delantero también se movió detrás del mismo multiplexor).

`v8-logica-completa` no se toca. Cuando esto quede validado, se promueve
junto con el resto a `firmware-esp32/` y `raspberry-pi/`.

## Mapa de la tira (8 LED, con equipo ya elegido)

| LED | Muestra |
|---|---|
| 0 | Equipo — **siempre fijo**, rojo o azul |
| 1 | QTR izquierdo — gris tenue = sin señal útil (hoy: siempre, hardware averiado), morado = borde detectado, verde tenue = pista libre |
| 2 | QTR derecho — mismos colores que el 1 |
| 3 | Color delantero — mismo código que el LED RGB de v8 (gris=piso, amarillo, rojo, azul, apagado=negro/desconocido) |
| 4 | Color trasero — gris muy tenue fijo si `kSensorTraseroInstalado=false`; si está instalado, mismo código que el 3 |
| 5 | Cámara ve la bandera y ToF en rango — verde = las dos condiciones (el requisito real para cerrar el gripper), azul tenue = solo cámara, apagado = ninguna |
| 6 | Gripper — blanco tenue=abierto, amarillo=cerrado sobre la caja, verde=cerrado con la bandera, cian=soltando despacio |
| 7 | Fase / error — mismo código que el LED RGB de v8 (blanco tenue=buscando, amarillo parpadeando=ajustando, verde=agarrada/parada de banco, rojo parpadeando=fallo, morado=borde) |

**Mientras el switch de equipo está en posición 0** (esperando), las 8 luces
corren una animación de espera en vez del mapa de arriba —
`Mission::kIdleModo`: `SCANNER` (magenta, un LED va y vuelve, por defecto) o
`RAINBOW` (arcoíris con los 8 encendidos).

**Al terminar la misión de verdad** (bandera soltada en la zona propia,
`Phase::TERMINADO`) la tira corre `theaterChaseRainbow` en bucle sin parar.
Las paradas de banco (`FIN_M1`/`FIN_M2`/`FIN_M3`, para pruebas cortas de
`kBancoSoloPaso7`/`kBancoPararTrasAgarrar`) se quedan con el verde fijo de
siempre — así no se confunde "banco" con "misión completa" al depurar.

Todos los parámetros (brillo, modo de espera, tiempos) están en
`Mission::`, al inicio de `src/main.cpp`, con su explicación.

## Reinicio manual tras terminar

Una vez la misión termina de verdad (`Phase::TERMINADO`, la tira celebrando
con `theaterChaseRainbow`), si el switch de equipo vuelve a la posición 0
(centro) por al menos `Mission::kReinicioDebounceMs` (400 ms), el ESP32 se
reinicia solo (`esp_restart()`) — así se puede correr otra misión sin tocar
el botón de reset ni el USB. Fuera de `Phase::TERMINADO` el switch no se
lee para nada (igual que siempre: solo se lee una vez, al arrancar).
`Mission::kReiniciarConSwitchEnCero = false` lo desactiva.

**Sobre avisarle a la Raspberry Pi que va a haber un reinicio** (lo que
Montse preguntó el 2026-09-27, sin decidir si hacía falta): revisé
`raspberry-pi/scripts/avisar_bandera_v8.py` y **ya reconecta solo** si
pierde el puerto serie (reintenta en silencio, sin caerse) — que es
exactamente lo que pasa cuando el ESP32 se reinicia: el USB nativo se
desconecta y reconecta un instante. No hace falta mandarle ningún aviso
especial: la Pi ya tolera el reinicio del ESP32 sin ayuda. Si algún día se
ve que la Pi SÍ se confunde con esto en pista, ahí se agrega el aviso, pero
hoy no parece necesario.

## Sensor trasero + multiplexor

`Mission::kSensorTraseroInstalado` (por defecto **`false`**): con esto en
`false`, el firmware se comporta EXACTAMENTE como v8 — ignora el
multiplexor y el trasero por completo, y el LED4 queda fijo en gris muy
tenue. Ponerlo en `true` **solo** después de confirmar con un escáner I2C
que el multiplexor (0x71) y el TCS34725 trasero (canal 1) responden — ver
[`hardware/conexiones-esp32-s3.md`](../../hardware/conexiones-esp32-s3.md),
sección de sensores de color, para el cableado y el porqué de la
dirección 0x71.

El color trasero **no participa en la lógica de la misión**, solo se
muestra en el LED4 — no hay ningún paso de los 12 que dependa de él.

## Uso

`pio run -t upload --upload-port COM11` y `pio device monitor --port COM11`
(log por UART, mismo puerto que el flasheo). Banco aislado de la tira, sin
el resto de la misión: [`pruebas-platformio/10-tira-ws2812/`](../../pruebas-platformio/10-tira-ws2812/).

| Estado | Nota |
|---|---|
| Compila | sí (2026-09-27) |
| Tira en misión real | sin probar todavía — pendiente correr la misión completa y ver el mapa en pista |
| Sensor trasero | sin probar — `kSensorTraseroInstalado=false` hasta confirmar el cableado del multiplexor |
