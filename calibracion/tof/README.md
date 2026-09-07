# Calibración del ToF (VL53L1X)

Herramienta de banco para encontrar a qué distancia (mm) cerrar el gripper
sobre la bandera. Imprime la distancia en vivo por consola y deja abrir/
cerrar el gripper a mano por comando, para correlacionar el número con un
agarre real.

## Hardware necesario

- 1x VL53L1X (bus I2C 0, mismos pines que `firmware-esp32/`)
- 1x PCA9685 + el servo del gripper (bus I2C 1)
- **Nada más** — sin motores, sin QTR, sin sensores de color.

## Uso

```bash
cd firmware
pio run -t upload
pio device monitor
```

La consola imprime la distancia 10 veces por segundo. Sostén la bandera (o
lo que vayas a agarrar) frente al sensor a distintas distancias y observa
el número. Cuando quieras probar si a esa distancia el gripper agarra bien:

- `C` + Enter → cierra el gripper (ángulo de "agarrar bandera")
- `O` + Enter → lo vuelve a abrir

Repite a varias distancias hasta encontrar la que agarre mejor y de forma
más consistente. Ese número es el que va a `distancia_agarre_mm` en
[`raspberry-pi/src/athena/config.py`](../../raspberry-pi/src/athena/config.py)
(hoy en 120.0, sin validar contra el sensor real) y al equivalente que use
cualquier standalone que agregue esta fase.

## Nota sobre el bus compartido con el TCS34725 delantero

El VL53L1X arranca siempre respondiendo en la dirección fija `0x29` — la
misma que usa el TCS34725 delantero, en el mismo bus I2C nº0 en el robot
real. Este banco lo reasigna a `0x30` apenas sale de reset (con `XSHUT` en
reset mientras tanto), así que no hace falta tener el TCS34725 conectado
para esta prueba — pero si lo tienes puesto de todas formas, no hay
conflicto: la reasignación ocurre antes de que nada más intente hablarle
al bus.
