# Firmware ESP32-S3 AUTÓNOMO — v5-agarrar-bandera

Avanza en línea recta, usa el **ToF (VL53L1X)** para saber cuándo la
bandera está al alcance del gripper, se **detiene por completo** para
medir con precisión, ajusta su posición con **pasos chicos** si hace
falta, cierra el gripper solo cuando confirma el rango de agarre **de
forma sostenida**, y se aleja con la bandera agarrada.

**Sin Raspberry Pi, sin cámara**: "detectar la bandera" acá es 100% el ToF
viendo un objeto a la distancia de agarre — no hay reconocimiento de color
ni de forma (eso lo exige el reglamento vía cámara + Edge Impulse en la
misión real, `raspberry-pi/`). Acá solo se prueba la mecánica de acercarse
y agarrar, con el objeto ya puesto a mano frente al robot.

## Distancia de agarre: 56–62 mm, confirmada en banco

Con [`calibracion/tof/`](../../calibracion/tof/README.md), sosteniendo la
bandera roja y la azul a mano: ambas agarraron bien en ~60 mm. Un solo
umbral compartido alcanza para las dos — no hace falta uno por color.

## Secuencia de misión

0. `ARRANQUE`: cuenta regresiva de 3 s para ubicar el robot y la bandera.
1. `AVANCE_GRUESO`: avanza recto hasta que el ToF dé una lectura válida
   ≤150 mm (margen generoso antes del rango de agarre, para no pasarse).
2. `DETENER_PARA_MEDIR` / `PASO_AJUSTE` (se alternan): el robot para por
   completo, espera a que el chasis se asiente y el ToF refresque, y mide.
   Si no está en 56–62 mm, da **un paso chico** (adelante si está lejos,
   atrás si está cerca) y vuelve a parar a medir. Se repite hasta leer el
   rango **3 veces seguidas** (no una lectura suelta) — recién ahí se
   considera confirmado.
3. `CERRAR_GRIPPER`: cierra sobre la bandera, con el robot ya quieto desde
   varios ciclos antes.
4. `GIRAR` / `ALEJARSE`: gira ~90° y avanza, para demostrar que puede
   transportar la bandera agarrada, no solo sujetarla en el sitio.
5. `TERMINADO`.

Si se agotan 40 pasos de ajuste sin confirmar el rango (la bandera se
movió, o el ToF da problemas), pasa a `FALLO_AJUSTE` y el LED lo avisa en
vez de seguir empujando indefinidamente.

## LED — indicador de fase

| Fase | LED |
|---|---|
| Buscando (avance grueso) | Blanco tenue fijo |
| Ajustando (parado/paso chico) | Amarillo parpadeando |
| Bandera agarrada / alejándose | Verde fijo |
| Terminado | Verde fijo |
| Fallo (se agotaron los pasos) | Rojo parpadeando |

## Por qué parar por completo antes de medir, y por qué en pasos chicos

Pedido explícito: el robot tiene que estar **completamente quieto** al
cerrar el gripper — cerrar a mitad de un frenado arriesga cerrar sobre el
aire o empujar la bandera antes de agarrarla. El ToF nunca se lee en
movimiento, y el gripper nunca se cierra sin antes confirmar el rango con
el robot ya detenido.

## "Constantemente en rango", no una lectura suelta

Una sola lectura del ToF en 56–62 mm puede ser ruido. Se exigen 3 lecturas
seguidas en rango, cada una con el robot ya asentado, antes de cerrar.

## Sin TCS34725 en esta variante

El ToF y el TCS34725 delantero comparten el bus I2C nº0 (misma dirección
fija 0x29), y la reasignación de dirección que los separaría **no está
funcionando todavía** (ver
[`calibracion/tof/README.md`](../../calibracion/tof/README.md), hallazgo
del 2026-09-07). Esta variante, igual que esa herramienta de banco, asume
que el ToF es el único dispositivo en el bus 0.

## Hardware necesario

- 2x L298N (4 motores)
- 1x VL53L1X (I2C bus 0) — solo, sin el TCS34725 delantero
- 1x PCA9685 + el servo del gripper (I2C bus 1)
- LED RGB
- Switch de 3 posiciones (solo como traba de seguridad)

**NADA** de QTR ni sensores de color en esta variante.

## Uso

```bash
pio run -t upload
pio device monitor
```

El robot espera con el switch en el centro. Muévelo a cualquier posición
para armarlo y arrancar la cuenta regresiva de 3 s. Pon la bandera a
1-1.5 m por delante, alineada con el ToF.
