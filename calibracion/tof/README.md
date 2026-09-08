# Calibración del ToF (VL53L1X) + TCS34725 delantero

Herramienta de banco para encontrar a qué distancia (mm) cerrar el gripper
sobre la bandera, y a la vez validar que el ToF y el sensor de color
delantero conviven bien en el robot. Imprime distancia + color en vivo por
consola y deja abrir/cerrar el gripper a mano por comando, para
correlacionar el número con un agarre real.

## Hardware necesario

- 1x VL53L1X — bus I2C 0 (GPIO8/9), solo, sin compartir bus con nadie
- 1x TCS34725 delantero — bus I2C 1 (GPIO47/48), junto al PCA9685
- 1x PCA9685 + el servo del gripper — bus I2C 1 (GPIO47/48)
- Sin motores, sin QTR, sin TCS34725 trasero (ver más abajo por qué)

## Uso

```bash
cd firmware
pio run -t upload
pio device monitor
```

La consola imprime distancia y color juntos, 10 veces por segundo. Sostén
la bandera (o lo que vayas a agarrar) frente al ToF a distintas distancias
y observa el número. Cuando quieras probar si a esa distancia el gripper
agarra bien:

- `C` + Enter → cierra el gripper (ángulo de "agarrar bandera")
- `O` + Enter → lo vuelve a abrir

Repite a varias distancias hasta encontrar la que agarre mejor y de forma
más consistente. Ese número es el que va a `distancia_agarre_mm` en
[`raspberry-pi/src/athena/config.py`](../../raspberry-pi/src/athena/config.py)
(hoy en 120.0, sin validar contra el sensor real) y al equivalente que use
cualquier standalone que agregue esta fase.

## El problema del bus compartido con el TCS34725 delantero — RESUELTO 2026-09-08 (de verdad)

El VL53L1X y el TCS34725 arrancan los DOS en la misma dirección fija
`0x29`. Ninguno de los dos tiene pines de selección de dirección (A0-A2) —
esos pines existen en un multiplexor I2C (TCA9548A), no en estos chips.

**Historial completo, incluyendo un "resuelto" que no lo era:**

1. **2026-09-07**: se intentó reasignar el ToF a `0x30` llamando
   `setAddress()` ANTES de `init()`, apenas 2 ms después de soltar `XSHUT`
   — demasiado pronto, el chip todavía no había terminado su arranque
   interno de firmware. No funcionó.

2. **2026-09-08, primer y segundo intento**: se reordenó a `init()` →
   `setAddress()` y pareció funcionar perfecto — el barrido mostraba el
   chip movido a `0x30`, lecturas estables cruzando toda la zona de agarre
   (56-62 mm). Se documentó como "RESUELTO".
   **Error**: esas pruebas se corrieron con el TCS34725 delantero **sin
   corriente** (un cable de 3.3V se había quedado desconectado sin
   notarlo, de una prueba anterior). Nunca hubo un segundo dispositivo de
   verdad compitiendo en `0x29` — el "resuelto" fue prematuro.

3. **2026-09-08, tercer intento (el real)**: con el TCS34725 delantero ya
   con corriente de verdad (LED del sensor encendido, confirmado a simple
   vista), el `init()` del ToF **falló por completo** — no solo
   `setAddress()`, la comunicación básica. Reproducido dos veces seguidas.
   **Causa real**: el TCS34725 no tiene pin de reset/apagado — en cuanto
   tiene corriente, contesta en `0x29` todo el tiempo, sin que nada lo
   pueda silenciar por software. El VL53L1X, para su propio `init()`,
   también necesita hablarle a `0x29` (su dirección de fábrica) antes de
   poder reasignarse. Con los DOS contestando `0x29` a la vez, cualquier
   transacción por ese bus recibe respuesta simultánea de ambos chips y
   se corrompe. El orden de llamadas nunca fue el problema real.

4. **Fix adoptado**: separar los buses físicamente, no por dirección.
   - **TCS34725 delantero**: recableado del bus I2C nº0 (GPIO8/9, donde
     vivía con el ToF) al bus I2C nº1 (GPIO47/48, donde ya vive el
     PCA9685 — dirección `0x40`, sin choque con `0x29`).
   - **TCS34725 trasero**: desconectado por completo — el bus 1 tampoco
     puede tener dos dispositivos en `0x29` a la vez.
   - **VL53L1X**: se queda solo en el bus 0, sin nadie con quien chocar —
     ya no hace falta reasignarle dirección para nada, se queda en su
     `0x29` de fábrica.

   **Validado en banco**: barrido del bus 0 muestra solo `0x29` (el ToF);
   barrido del bus 1 muestra `0x29` y `0x40` (TCS delantero + PCA9685).
   Los dos sensores inicializan y leen en simultáneo sin ningún
   "SENSOR NO RESPONDE" — probado con el ToF siguiendo una mano (78 mm →
   776 mm) mientras el TCS delantero se movía sobre distintas superficies
   al mismo tiempo, sin ninguna lectura corrupta de ningún lado.

## Consecuencia para el resto del proyecto

Este bench tool queda como la referencia del cableado correcto para
cualquier firmware que necesite el ToF y el TCS34725 delantero a la vez:
**ToF solo en el bus 0, TCS34725 delantero junto al PCA9685 en el bus 1,
TCS34725 trasero desconectado** (o, si hiciera falta recuperarlo más
adelante, necesitaría su propio multiplexor I2C — no hay forma de
compartir bus con otro `0x29` sin uno). `firmware-esp32/` (la misión
real) hoy no tiene el sensor delantero conectado, así que no le afecta
todavía, pero si se reincorpora, este es el cableado a seguir.

Los umbrales de color usados acá (`UmbralColor::` en
`firmware/src/main.cpp`) son los mismos de
[`../color/detector-tcs/src/main.cpp`](../color/detector-tcs/src/main.cpp),
calibrados con el TCS34725 en el bus 0 -- si el bus nuevo le cambia el
ruido eléctrico percibido, la primera señal de alerta sería confusiones
entre GRIS y AMARILLO como las que ya pasaron el 2026-09-07 durante la
calibración original.
